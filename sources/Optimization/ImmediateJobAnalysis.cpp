
#include "sources/Optimization/ImmediateJobAnalysis.h"

namespace OrderOptDAG_SPACE
{
  std::vector<JobCEC> GetVectorFromSet(const std::unordered_set<JobCEC> &record)
  {
    std::vector<JobCEC> vec;
    vec.reserve(record.size());
    for (auto itr = record.begin(); itr != record.end(); itr++)
    {
      vec.push_back(*itr);
    }
    return vec;
  }

  bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                const VectorDynamic &startTimeVector, double tolerance)
  {
    if (instCurr.type == 's')
    {
      if (instCompare.type == 's')
      {
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance)
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance)
        {
          return true;
        }
      }
    }
    else
    { // instCurr.type=='f'
      if (instCompare.type == 's')
      {
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance)
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance)
        {
          return true;
        }
      }
    }

    return false;
  }
  // returns whether instCompare is an immediate forward adjacent job to instCurr
  bool WhetherImmediateForwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                       const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                       const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                       double tolerance)
  {
    if (instCurr.type == 's')
    {
      if (instCompare.type == 's')
      {
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobStartInstancePosition(instCompare.job) <
                jobOrder.GetJobStartInstancePosition(instCurr.job))
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobStartInstancePosition(instCompare.job) <
                jobOrder.GetJobFinishInstancePosition(instCurr.job))
        {
          return true;
        }
      }
    }
    else
    { // instCurr.type=='f'
      if (instCompare.type == 's')
      {
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobStartInstancePosition(instCompare.job) <
                jobOrder.GetJobFinishInstancePosition(instCurr.job))
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobFinishInstancePosition(instCompare.job) <
                jobOrder.GetJobFinishInstancePosition(instCurr.job))
        {
          return true;
        }
      }
    }

    return false;
  }

  void AddImmediateForwardInstance(LLint jobInstIndex, TimeInstance instCurrJob, SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &record)
  {
    for (int i = jobInstIndex - 1; i >= 0; i--)
    {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateForwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder))
      {
        if (record.find(instIte.job) == record.end())
        {
          record.insert(instIte.job);
          FindForwardAdjacentJob(instIte.job, jobOrder, tasksInfo, startTimeVector, record);
        }
        else
          break;
      }
      else
        break;
    }
  }

  // previous adjacent job means the jobs whose finish time equals the start time of jobCurr
  // in-place update to record
  // TODO: update FindBackwardAdjacentJob
  void FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &record)
  {
    LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
    TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
    AddImmediateForwardInstance(jobStartIndex, instCurrJobStart, jobOrder, tasksInfo, startTimeVector, record);

    LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
    TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
    AddImmediateForwardInstance(jobFinishIndex, instCurrJobFinish, jobOrder, tasksInfo, startTimeVector, record);
  }

  std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                             const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                             const VectorDynamic &startTimeVector)
  {
    std::unordered_set<JobCEC> record;
    record.reserve(4 * 2); // actually, cannot be more than #core*2
    FindForwardAdjacentJob(job, jobOrder, tasksInfo, startTimeVector, record);
    return GetVectorFromSet(record);
  }

  bool WhetherImmediateBackwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                        const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                        const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                        double tolerance)
  {
    if (instCurr.type == 's')
    {
      if (instCompare.type == 's')
      {
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobStartInstancePosition(instCompare.job) >
                jobOrder.GetJobStartInstancePosition(instCurr.job))
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobFinishInstancePosition(instCompare.job) >
                jobOrder.GetJobStartInstancePosition(instCurr.job))
        {
          return true;
        }
      }
    }
    else
    { // instCurr.type=='f'
      if (instCompare.type == 's')
      {
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobStartInstancePosition(instCompare.job) >
                jobOrder.GetJobFinishInstancePosition(instCurr.job))
        {
          return true;
        }
      }
      else
      { // instCompare.type=='f'
        if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                     GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
            jobOrder.GetJobFinishInstancePosition(instCompare.job) >
                jobOrder.GetJobFinishInstancePosition(instCurr.job))
        {
          return true;
        }
      }
    }

    return false;
  }

  void AddImmediateBackwardInstance(LLint jobInstIndex, TimeInstance instCurrJob, SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                    const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &record)
  {
    for (uint i = jobInstIndex + 1; i < jobOrder.size(); i++)
    {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateBackwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder))
      {
        if (record.find(instIte.job) == record.end())
        {
          record.insert(instIte.job);
          FindBackwardAdjacentJob(instIte.job, jobOrder, tasksInfo, startTimeVector, record);
        }
        else
          break;
      }
      else
        break;
    }
  }

  void FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                               const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                               const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &record)
  {
    LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
    TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
    AddImmediateBackwardInstance(jobStartIndex, instCurrJobStart, jobOrder, tasksInfo, startTimeVector, record);

    LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
    TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
    AddImmediateBackwardInstance(jobFinishIndex, instCurrJobFinish, jobOrder, tasksInfo, startTimeVector, record);
  }

  // 'backward' means finding the jobs whose index is larger than job, i.e., -->
  std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                              const VectorDynamic &startTimeVector)
  {
    std::unordered_set<JobCEC> record;
    record.reserve(tasksInfo.length); // actually, cannot be more than #core*2
    FindBackwardAdjacentJob(job, jobOrder, tasksInfo, startTimeVector, record);
    return GetVectorFromSet(record);
  }
} // namespace OrderOptDAG_SPACE