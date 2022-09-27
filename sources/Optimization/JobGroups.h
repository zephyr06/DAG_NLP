#pragma once
#include "unordered_set"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/RelocateStartTimeVector.h"

namespace DAG_SPACE
{

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
            if ((*itr)->error(initialEstimateFG) != 0)
            {
                gtsam::KeyVector keys = (*itr)->keys();
                indexPairsWithError.push_back(KeyVector2JobCECVec(keys));
                itr->get()->printKeys();
                std::cout << (*itr)->error(initialEstimateFG) << std::endl;
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

        bool exist(const JobCEC &job)
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

        // return true if at least one job exists in the job group and given job vectors
        bool existOverlap(std::vector<JobCEC> &jobs)
        {
            for (size_t i = 0; i < jobs.size(); i++)
            {
                if (exist(jobs[i]))
                    return true;
            }
            return false;
        }

        // probably not gonna be used
        bool existVanishGradient(GradientVanishPairs &gvp)
        {
            if (gvp.size() == 0)
                return false;

            for (auto &keyVec : gvp.vanishPairs_)
            {
                bool whetherMatch = true;
                for (auto &key : keyVec)
                {
                    gtsam::Symbol s(key);
                    JobCEC job{AnalyzeKey(s)};
                    if (!exist(job))
                    {
                        whetherMatch = false;
                        break;
                    }
                }
                if (whetherMatch)
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

        size_t size() { return jobs_.size(); }
    };

    std::vector<JobGroup>
    CreateJobGroups(std::vector<std::vector<JobCEC>> &jobPairsWithError)
    {
        std::vector<JobGroup> jobGroups;
        if (jobPairsWithError.size() == 0)
        {
            return jobGroups;
        }
        jobGroups.reserve(jobPairsWithError.size());
        for (size_t i = 0; i < jobPairsWithError.size(); i++)
        {
            bool findMatchGroup = false;
            for (size_t j = 0; j < jobGroups.size(); j++)
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
        return jobGroups;
    }

    VectorDynamic AdjustIndexOrderWithinEachGroup(std::vector<JobGroup> jobGroups, VectorDynamic initialEstimate)
    {
        for (JobGroup &group : jobGroups)
        {
            // IF only DBF and DDL constraints are considered
        }
        return initialEstimate;
    }

    VectorDynamic JobGroupsOptimize(VectorDynamic &initialEstimate, TaskSetInfoDerived &tasksInfo)
    {
        return initialEstimate;
        // std::vector<LLint> FindJobIndexWithError(...) e.g., {3,1,4,8}
        // std::vector<std::vector<LLint>> GroupIndexWithError(...) e.g., {{3,1,4,8}}
        // VectorDynamic AdjustIndexOrderWithinEachGroup(std::vector<std::vector<LLint>> groupIndex, VectorDynamic initialEstimate)
    }

} // namespace DAG_SPACE