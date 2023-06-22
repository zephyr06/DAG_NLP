#include "sources/Factors/RTDA_Factor.h"

namespace OrderOptDAG_SPACE
{

  RTDA GetMaxRTDA(const std::vector<RTDA> &resVec)
  {
    RTDA maxRTDA;
    for (const RTDA &item : resVec)
    {
      maxRTDA.reactionTime = std::max(item.reactionTime, maxRTDA.reactionTime);
      maxRTDA.dataAge = std::max(item.dataAge, maxRTDA.dataAge);
    }
    return maxRTDA;
  }

  std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo,
                                         const std::vector<int> &causeEffectChain, const VectorDynamic &x,
                                         double precision)
  {
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
        while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) + precision < currentJobFT)
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
      resVec[startInstanceIndex].reactionTime =
          GetFinishTime(firstJob, x, tasksInfo) -
          GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);

      // update data age
      JobCEC firstReactLastJob = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
      if (startInstanceIndex > 0 && firstReactionMap[firstReactLastJob] != firstJob && firstJob.jobId > 0)
      {
        JobCEC lastReaction = firstJob;
        lastReaction.jobId--;
        resVec[startInstanceIndex - 1].dataAge =
            GetStartTime(lastReaction, x, tasksInfo) + tasks[lastReaction.taskId].executionTime -
            GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
      }
    }
    return resVec;
  }

  std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo,
                                         const std::vector<int> &causeEffectChain, const gtsam::Values &x)
  {
    VectorDynamic stvAfter = GenerateVectorDynamic(tasksInfo.variableDimension);
    for (int i = 0; i < tasksInfo.N; i++)
    {
      for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
      {
        LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
        gtsam::Symbol key = GenerateKey(i, j);
        VectorDynamic aaa = x.at<VectorDynamic>(key);
        stvAfter(index_overall, 0) = x.at<VectorDynamic>(key)(0, 0);
      }
    }
    return GetRTDAFromSingleJob(tasksInfo, causeEffectChain, stvAfter);
  }

  RTDA GetMaxRTDA(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain,
                  const VectorDynamic &startTimeVector)
  {
    std::vector<RTDA> rtdaVec = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, startTimeVector);
    RTDA maxRTDA = GetMaxRTDA(rtdaVec);
    return maxRTDA;
  }

  double ObjRTDA(const RTDA &rtda) { return rtda.reactionTime + rtda.dataAge; }
  double ObjRTDA(const std::vector<RTDA> &rtdaVec)
  {
    return ObjRT(rtdaVec) + ObjDA(rtdaVec);
  }
  double ObjRT(const std::vector<RTDA> &rtdaVec)
  {
    double res = 0;
    for (auto &r : rtdaVec)
      res += r.reactionTime;
    return res;
  }
  double ObjDA(const std::vector<RTDA> &rtdaVec)
  {
    double res = 0;
    for (auto &r : rtdaVec)
      if (r.dataAge != -1)
        res += r.dataAge;
    return res;
  }
  // **********************************************************************
  std::unordered_map<JobCEC, std::vector<JobCEC>>
  GetRTDAReactChainsFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain,
                                  const VectorDynamic &x)
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
        LLint jobIndex = 0; // TODO: test efficiency and consider whether make this faster
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
  // TODO: rm processorNum and chainIndex from arguments?
  std::unordered_map<JobCEC, std::vector<JobCEC>>
  GetReactionChainMap(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                      int processorNum, const std::vector<int> &causeEffectChain, int chainIndex)
  {
    std::unordered_map<JobCEC, std::vector<JobCEC>> firstReactionChainMap;
    LLint hyperPeriod = tasksInfo.hyperPeriod;
    const TaskSet &tasks = tasksInfo.tasks;
    LLint totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;

    for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
    {
      JobCEC firstJob = {causeEffectChain[0], (startInstanceIndex)};
      std::vector<JobCEC> react_chain;
      react_chain.reserve(causeEffectChain.size());
      react_chain.push_back(firstJob);
      for (uint j = 1; j < causeEffectChain.size(); j++)
      {
        LLint instIndexFirstJob = jobOrder.GetJobFinishInstancePosition(firstJob);
        LLint jobIndex = 0;
        while (true)
        {
          JobCEC jobCurr{causeEffectChain[j], jobIndex};
          if (jobOrder.GetJobStartInstancePosition(jobCurr) > instIndexFirstJob)
            break;
          jobIndex++;
          if (jobIndex > 10000)
          {
            CoutError("didn't find a match in GetJacobianCauseEffectChainOrg!");
          }
        }
        firstJob = {causeEffectChain[j], jobIndex};
        react_chain.push_back(firstJob);
      }

      JobCEC startJobCurr(causeEffectChain[0], startInstanceIndex);
      firstReactionChainMap[startJobCurr] = react_chain;
    }
    return firstReactionChainMap;
  }

  std::unordered_map<JobCEC, std::vector<JobCEC>>
  GetDataAgeChainMap(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                     int processorNum, const std::vector<int> &causeEffectChain, int chainIndex)
  {
    std::unordered_map<JobCEC, std::vector<JobCEC>> lastReadingChainMap;
    LLint totalStartJobs = tasksInfo.hyperPeriod / tasksInfo.tasks[causeEffectChain.back()].period;

    for (LLint startInstanceIndex = 0; startInstanceIndex < totalStartJobs; startInstanceIndex++)
    {
      JobCEC lastJob = {causeEffectChain.back(), startInstanceIndex};
      std::vector<JobCEC> job_chain;
      job_chain.reserve(causeEffectChain.size());
      job_chain.push_back(lastJob);
      for (int j = causeEffectChain.size() - 2; j >= 0; j--)
      {
        LLint instIndexFirstJob = jobOrder.GetJobStartInstancePosition(lastJob);
        LLint max_last_job_start_time = GetDeadline(lastJob, tasksInfo) - GetExecutionTime(lastJob, tasksInfo);
        int max_prev_job_index = max_last_job_start_time / tasksInfo.tasks[causeEffectChain[j]].period + 1; // probably does not need +1
        LLint jobIndex = max_prev_job_index;
        while (true)
        {
          JobCEC jobCurr{causeEffectChain[j], jobIndex};
          if (jobOrder.GetJobFinishInstancePosition(jobCurr) < instIndexFirstJob)
            break;
          jobIndex--;
          if (jobIndex < -10000)
          {
            CoutError("didn't find a match in GetDataAgeChainMap!");
          }
        }
        lastJob = {causeEffectChain[j], jobIndex};
        job_chain.push_back(lastJob);
      }

      std::reverse(job_chain.begin(), job_chain.end());
      JobCEC startJobCurr(causeEffectChain.back(), startInstanceIndex);
      lastReadingChainMap[startJobCurr] = job_chain;
    }
    return lastReadingChainMap;
  }
} // namespace OrderOptDAG_SPACE