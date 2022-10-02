#pragma once
#include "unordered_set"
#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"

namespace DAG_SPACE
{

    gtsam::Values GenerateInitialFG(const VectorDynamic &startTimeVector, const TaskSetInfoDerived &tasksInfo)
    {
        gtsam::Values initialEstimateFG;
        gtsam::Symbol key('a', 0); // just declare the variable

        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables.at(i)); j++)
            {
                // LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
                gtsam::Symbol key = GenerateKey(i, j);
                VectorDynamic v = GenerateVectorDynamic(1);
                v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);

                initialEstimateFG.insert(key, v);
            }
        }
        return initialEstimateFG;
    }

    VectorDynamic AlignVariablesF2I(VectorDynamic &x, double threshold)
    {
        VectorDynamic y = x;
        for (long int i = 0; i < y.rows(); i++)
        {
            if (std::abs(y(i) - round(y(i))) < threshold)
            {
                y(i) = round(y(i));
            }
        }
        return y;
    }

    void PrintKeyVector(gtsam::KeyVector &vec)
    {
        for (uint i = 0; i < vec.size(); i++)
        {
            gtsam::Symbol{vec[i]}.print();
            std::cout << ", ";
        }
        std::cout << endl;
    }

    std::vector<JobCEC> KeyVector2JobCECVec(gtsam::KeyVector &keys)
    {
        std::vector<JobCEC> res;
        res.reserve(keys.size());
        for (uint i = 0; i < keys.size(); i++)
        {
            auto s = AnalyzeKey(gtsam::Symbol{keys[i]});
            res.push_back(JobCEC{s.first, s.second});
        }
        return res;
    }

    std::vector<std::vector<JobCEC>> FindJobIndexWithError(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo, NonlinearFactorGraph &graph)
    {
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        std::vector<std::vector<JobCEC>> indexPairsWithError;
        // go through each factor
        for (auto itr = graph.begin(); itr != graph.end(); itr++)
        {
            if ((*itr)->error(initialEstimateFG) > 1e-2)
            {
                gtsam::KeyVector keys = (*itr)->keys();
                indexPairsWithError.push_back(KeyVector2JobCECVec(keys));
                if (debugMode == 1)
                {
                    itr->get()->printKeys();
                    std::cout << (*itr)->error(initialEstimateFG) << std::endl;
                    // if ((*itr)->error(initialEstimateFG) >= 54449)
                    // {
                    //     // int a = 1;
                    // }
                }
            }
        }
        return indexPairsWithError;
    }

    class JobGroup
    {
    public:
        std::unordered_set<JobCEC> jobs_;
        JobGroup(std::vector<JobCEC> &jobs)
        {
            insert(jobs);
        }

        std::vector<JobCEC> sort(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo, string sortType = "deadline")
        {
            std::vector<JobCEC> jobVec;
            jobVec.reserve(size());
            for (auto ite = jobs_.begin(); ite != jobs_.end(); ite++)
            {
                jobVec.push_back(*ite);
            }
            if (sortType == "deadline")
            {
                auto compareFunc = [&](JobCEC &j1, JobCEC &j2) -> bool
                {
                    return GetDeadline(j1, tasksInfo) < GetDeadline(j2, tasksInfo);
                };
                std::sort(jobVec.begin(), jobVec.end(), compareFunc);
            }
            else
                CoutError("Unrecognized sortType in JobGroup!");
            return jobVec;
        }

        bool exist(const JobCEC &job) const
        {
            return jobs_.find(job) != jobs_.end();
        }

        void insert(std::vector<JobCEC> &jobs)
        {
            for (size_t i = 0; i < jobs.size(); i++)
            {
                if (jobs_.find(jobs[i]) == jobs_.end())
                {
                    jobs_.insert(jobs[i]);
                }
            }
        }
        void insert(JobGroup &jobs)
        {
            for (auto itr = jobs.jobs_.begin(); itr != jobs.jobs_.end(); itr++)
            {
                if (!exist(*itr))
                {
                    jobs_.insert(*itr);
                }
            }
        }

        // return true if at least one job exists in the job group and given job vectors
        bool existOverlap(std::vector<JobCEC> &jobs) const
        {
            for (size_t i = 0; i < jobs.size(); i++)
            {
                if (exist(jobs[i]))
                    return true;
            }
            return false;
        }
        // return true if at least one job exists in the job group and given job vectors
        bool existOverlap(JobGroup &jobs) const
        {
            for (auto itr = jobs.jobs_.begin(); itr != jobs.jobs_.end(); itr++)
            {
                if (exist(*itr))
                    return true;
            }
            return false;
        }

        JobCEC findRightJob(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo)
        {
            if (size() == 0)
            {
                CoutError("No right most job because job group is empty!");
            }
            double finishTimeMax = -100;
            JobCEC rightMostJob;
            for (auto ite = jobs_.begin(); ite != jobs_.end(); ite++)
            {
                double finishTime = GetFinishTime(*(ite), startTimeVector, tasksInfo);
                if (finishTime > finishTimeMax)
                {
                    finishTimeMax = finishTime;
                    rightMostJob = *ite;
                }
            }
            return rightMostJob;
        }

        VectorDynamic SwitchRightJob(VectorDynamic startTimeVector, TaskSetInfoDerived &tasksInfo)
        {
            JobCEC rightMostJob = findRightJob(startTimeVector, tasksInfo);
            double rPeriodBegin = GetActivationTime(rightMostJob, tasksInfo);
            double rDeadline = GetDeadline(rightMostJob, tasksInfo);
            // exam whether rightmostjob violates deadline, if not, then no need to swap, and let vanish gradient/RTDA's swap to take care of this case.
            if (GetFinishTime(rightMostJob, startTimeVector, tasksInfo) <= rDeadline)
            {
                return startTimeVector;
            }

            std::vector<JobCEC> sortedJobs = sort(startTimeVector, tasksInfo, "deadline");
            // go through jobs from largest deadline to smallest deadline
            for (int i = static_cast<int>(size()) - 1; i >= 0; i--)
            {
                JobCEC jobCurr = sortedJobs[i];
                if (jobCurr == rightMostJob)
                    continue;

                // jobcurr's deadline should be larger than rightMostJob's deadline
                // rightMostJob's begin time should be smaller than jobCurr's smaller time
                // if (GetDeadline(jobCurr, tasksInfo) > rDeadline)
                if (GetDeadline(jobCurr, tasksInfo) > rDeadline && rPeriodBegin <= GetStartTime(jobCurr, startTimeVector, tasksInfo))
                {
                    swap(startTimeVector, IndexTran_Instance2Overall(jobCurr.taskId, jobCurr.jobId, tasksInfo.sizeOfVariables), IndexTran_Instance2Overall(rightMostJob.taskId, rightMostJob.jobId, tasksInfo.sizeOfVariables));
                    std::cout << "/n/n Actually do the swap !!!!/n/n";
                    return startTimeVector;
                }
            }

            // no good match found, randomly swap
            JobCEC jobCurr = sortedJobs[rand() % size()];
            swap(startTimeVector, IndexTran_Instance2Overall(jobCurr.taskId, jobCurr.jobId, tasksInfo.sizeOfVariables), IndexTran_Instance2Overall(rightMostJob.taskId, rightMostJob.jobId, tasksInfo.sizeOfVariables));
            return startTimeVector;
        }

        size_t size() { return jobs_.size(); }
    };

    std::vector<JobGroup> MergeJobGroup(std::vector<JobGroup> &jobGroups)
    {
        std::vector<bool> eraseRecord(jobGroups.size(), false);

        for (size_t i = 0; i < jobGroups.size(); i++)
        {
            JobGroup &jobGroupCurr = jobGroups[i];
            for (size_t j = i + 1; j < jobGroups.size(); j++)
            {
                if (jobGroups[j].existOverlap(jobGroupCurr))
                {
                    jobGroups[j].insert(jobGroupCurr);
                    eraseRecord[i] = true;
                    break;
                }
            }
        }
        std::vector<JobGroup> res;
        res.reserve(jobGroups.size());
        for (uint i = 0; i < eraseRecord.size(); i++)
        {
            if (!eraseRecord[i])
                res.push_back(jobGroups[i]);
        }
        return res;
    }

    std::vector<JobGroup>
    CreateJobGroups(std::vector<std::vector<JobCEC>> &jobPairsWithError)
    {
        std::vector<JobGroup> jobGroups;
        if (jobPairsWithError.size() == 0)
            return jobGroups;

        for (size_t i = 0; i < jobPairsWithError.size(); i++)
        {
            bool findMatchGroup = false;
            for (uint j = 0; j < jobGroups.size(); j++)
            {
                if (jobGroups[j].existOverlap(jobPairsWithError[i]))
                {
                    jobGroups[j].insert(jobPairsWithError[i]);
                    findMatchGroup = true;
                    break;
                }
            }
            if (!findMatchGroup)
                jobGroups.push_back(JobGroup(jobPairsWithError[i]));
        }

        jobGroups = MergeJobGroup(jobGroups);

        return jobGroups;
    }

    VectorDynamic JobGroupsOptimize(VectorDynamic &initialEstimate, TaskSetInfoDerived &tasksInfo, NonlinearFactorGraph &graph)
    {
        std::vector<std::vector<JobCEC>> jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
        std::vector<JobGroup> jobGroups = CreateJobGroups(jobPairsWithError);
        for (size_t i = 0; i < jobGroups.size(); i++)
            initialEstimate = jobGroups[0].SwitchRightJob(initialEstimate, tasksInfo);

        return initialEstimate;
    }

} // namespace DAG_SPACE