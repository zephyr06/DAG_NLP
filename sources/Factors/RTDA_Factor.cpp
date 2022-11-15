#include "sources/Factors/RTDA_Factor.h"

namespace OrderOptDAG_SPACE
{

    RTDA GetMaxRTDA(std::vector<RTDA> &resVec)
    {
        RTDA maxRTDA;
        for (auto &item : resVec)
        {
            maxRTDA.reactionTime = std::max(item.reactionTime, maxRTDA.reactionTime);
            maxRTDA.dataAge = std::max(item.dataAge, maxRTDA.dataAge);
        }
        return maxRTDA;
    }

    std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const VectorDynamic &x)
    {
        // gtsam::Values initialFG = GenerateInitialFG(x, tasksInfo);
        // return GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialFG);
        if (causeEffectChain.size() == 0)
        {
            return {RTDA(0, 0)};
        }
        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        LLint totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;
        RTDA res;
        std::vector<RTDA> resVec;
        resVec.reserve(totalStartJobs);
        for (LLint i = 0; i < totalStartJobs; i++)
        {
            resVec.push_back(RTDA{-1, -1});
        }

        std::unordered_map<JobCEC, JobCEC> firstReactionMap;

        for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
        {

            JobCEC firstJob = {causeEffectChain[0], (startInstanceIndex)};
            for (uint j = 1; j < causeEffectChain.size(); j++)
            {
                double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
                LLint jobIndex = 0;
                while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
                {
                    jobIndex++;
                    if (jobIndex > 10000)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                firstJob = {causeEffectChain[j], jobIndex};
            }

            JobCEC jj(causeEffectChain[0], startInstanceIndex);
            firstReactionMap[jj] = firstJob;
            resVec[startInstanceIndex].reactionTime = GetFinishTime(firstJob, x, tasksInfo) - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);

            // update data age
            JobCEC firstReactLastJob = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
            if (startInstanceIndex > 0 && firstReactionMap[firstReactLastJob] != firstJob && firstJob.jobId > 0)
            {
                JobCEC lastReaction = firstJob;
                lastReaction.jobId--;
                resVec[startInstanceIndex - 1].dataAge = GetStartTime(lastReaction, x, tasksInfo) + tasks[lastReaction.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
            }
        }
        return resVec;
    }

    std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const Values &x)
    {
        VectorDynamic stvAfter = GenerateVectorDynamic(tasksInfo.variableDimension);
        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
            {
                LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
                Symbol key = GenerateKey(i, j);
                VectorDynamic aaa = x.at<VectorDynamic>(key);
                stvAfter(index_overall, 0) = x.at<VectorDynamic>(key)(0, 0);
            }
        }
        return GetRTDAFromSingleJob(tasksInfo, causeEffectChain, stvAfter);
    }

    RTDA GetMaxRTDA(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, VectorDynamic &startTimeVector)
    {
        // TODO: improve efficiency by avoiding transforming vector to values
        gtsam::Values initialFG = GenerateInitialFG(startTimeVector, tasksInfo);
        std::vector<RTDA> rtdaVec = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialFG);
        RTDA maxRTDA = GetMaxRTDA(rtdaVec);
        return maxRTDA;
    }

    double ObjRTDA(const RTDA &rtda)
    {
        return rtda.reactionTime + rtda.dataAge;
    }
    double ObjRTDA(const std::vector<RTDA> &rtdaVec)
    {
        double res = 0;
        for (auto &r : rtdaVec)
            res += ObjRTDA(r);
        return res;
    }

    std::unordered_map<JobCEC, std::vector<JobCEC>> GetRTDAReactChainsFromSingleJob(
        const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const VectorDynamic &x)
    {
        std::unordered_map<JobCEC, std::vector<JobCEC>> firstReactionChainMap;

        if (causeEffectChain.size() == 0)
        {
            return firstReactionChainMap;
        }
        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        LLint totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;

        for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
        {
            JobCEC firstJob = {causeEffectChain[0], (startInstanceIndex)};
            std::vector<JobCEC> react_chain;
            react_chain.push_back(firstJob);
            for (uint j = 1; j < causeEffectChain.size(); j++)
            {
                double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
                LLint jobIndex = 0;
                while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
                {
                    jobIndex++;
                    if (jobIndex > 10000)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                firstJob = {causeEffectChain[j], jobIndex};
                react_chain.push_back(firstJob);
            }

            JobCEC jj(causeEffectChain[0], startInstanceIndex);
            firstReactionChainMap[jj] = react_chain;
        }
        return firstReactionChainMap;
    }
}