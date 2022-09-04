#include "unordered_map"

#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"

namespace DAG_SPACE
{
    using namespace std;
    struct JobCEC
    {
        int taskId;
        size_t jobId;
        JobCEC() : taskId(-1), jobId(0) {}
        JobCEC(int taskId, size_t jobId) : taskId(taskId), jobId(jobId) {}

        bool operator==(const JobCEC &other) const
        {
            return taskId == other.taskId && jobId == other.jobId;
        }
        bool operator!=(const JobCEC &other) const
        {
            return !(*this == other);
        }
    };
    template <>
    struct std::hash<JobCEC>
    {
        std::size_t operator()(const JobCEC &jobCEC) const
        {
            std::string str = std::to_string(jobCEC.taskId) + ", " + std::to_string(jobCEC.jobId);
            return std::hash<std::string>{}(str);
        }
    };

    // TODO: requires further test!
    double GetStartTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo.N)
        {
            CoutError("GetStartTime receives invalid jobCEC!");
        }
        int jobNumInHyperPeriod = tasksInfo.hyperPeriod / tasksInfo.tasks[jobCEC.taskId].period;

        double res = x.at<VectorDynamic>(GenerateKey(jobCEC.taskId, jobCEC.jobId % jobNumInHyperPeriod))(0) + jobCEC.jobId / jobNumInHyperPeriod * tasksInfo.hyperPeriod;
        return res;
    }

    inline double GetFinishTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }

    struct RTDA
    {
        double reactionTime;
        double dataAge;
        RTDA() : reactionTime(-1), dataAge(-1) {}
        RTDA(double r, double d) : reactionTime(r), dataAge(d) {}
    };

    RTDA GetMaxRTDA(std::vector<RTDA> &resVec)
    {
        RTDA maxRTDA;
        for (auto &item : resVec)
        {
            maxRTDA.reactionTime = max(item.reactionTime, maxRTDA.reactionTime);
            maxRTDA.dataAge = std::max(item.dataAge, maxRTDA.dataAge);
        }
        return maxRTDA;
    }

    std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const Values &x)
    {

        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        size_t totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;
        RTDA res;
        std::vector<RTDA> resVec;
        resVec.reserve(totalStartJobs);
        for (size_t i = 0; i < totalStartJobs; i++)
        {
            resVec.push_back(RTDA{-1, -1});
        }

        std::unordered_map<JobCEC, JobCEC> firstReactionMap;
        // Todo: Be careful! this termination condition is not consistent with Verucchi!
        for (size_t startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
        {

            JobCEC firstJob = {causeEffectChain[0], startInstanceIndex};
            JobCEC lastJob = {-1, 0};
            for (uint j = 1; j < causeEffectChain.size(); j++)
            {
                double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
                size_t jobIndex = 0;
                while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
                {
                    jobIndex++;
                    if (jobIndex > 100)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                firstJob = {causeEffectChain[j], jobIndex};
                if (jobIndex > 0)
                {
                    lastJob = {causeEffectChain[j], jobIndex - 1};
                }
                else
                {
                    CoutError("Should not gonna happen!");
                }
            }

            // TODO: Be careful about the last instance
            JobCEC jj(causeEffectChain[0], size_t(0));
            firstReactionMap[jj] = firstJob;
            resVec[startInstanceIndex].reactionTime = GetFinishTime(firstJob, x, tasksInfo) - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);

            // update data age
            JobCEC firstReactLastJob = JobCEC{causeEffectChain[0], size_t(startInstanceIndex - 1)};
            if (startInstanceIndex > 0 && firstReactionMap[firstReactLastJob] != firstJob)
            {
                JobCEC lastReaction = firstJob;
                lastReaction.jobId--;
                resVec[startInstanceIndex - 1].dataAge = GetStartTime(lastReaction, x, tasksInfo) + tasks[lastReaction.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
            }
        }
        return resVec;
    }

    void AddWholeRTDAFactor(NonlinearFactorGraph &graph,
                            TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain)
    {
        if (RtdaWeight == 0)
            return;

        std::vector<gtsam::Symbol> keysAll;
        keysAll.reserve(tasksInfo.length);
        for (uint i = 0; i < tasksInfo.tasks.size(); i++)
        {
            for (size_t j = 0; j < static_cast<size_t>(tasksInfo.hyperPeriod / tasksInfo.tasks[i].period); j++)
            {
                keysAll.push_back(GenerateKey(i, j));
            }
        }

        LambdaMultiKey f = [keysAll, tasksInfo, causeEffectChain](const Values &x)
        {
            auto RTDAVec = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, x);
            RTDA finalResult = GetMaxRTDA(RTDAVec);
            VectorDynamic res = GenerateVectorDynamic(1 * 2);
            res << finalResult.dataAge, finalResult.reactionTime;
            return res;
        };

        LLint errorDimensionRTDA = 2;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionRTDA, noiseModelSigma / RtdaWeight);

        graph.emplace_shared<MultiKeyFactor>(keysAll, f, 2, model);
    }
}