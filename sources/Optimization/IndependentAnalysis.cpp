
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/IndependentAnalysis.h"

namespace OrderOptDAG_SPACE
{
  // TODO: consider utilize startP and finishP
  // Assumption: start time of jobCurr in startTimeVector cannot move earlier
  bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
                                    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                    const VectorDynamic &startTimeVector)
  {
    // ,
    // std::unordered_set<JobCEC> &pathRecord, int &countPath
    // if (countPath > 10)
    //   return true;
    if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) - GetActivationTime(jobCurr, tasksInfo)) <
        1e-3)
      return false;
    else if (jobCurr == jobChanged)
    {
      // countPath++;
      return true;
    }

    std::vector<JobCEC> prevAdjacentJobs =
        FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    if (prevAdjacentJobs.size() == 0)
      return false; // consider more, should be false under assumptions  of startTimeVector

    for (auto &jobPrev : prevAdjacentJobs)
    {
      if (WhetherJobStartEarlierHelper(jobPrev, jobChanged, jobGroupMap, jobOrder, tasksInfo,
                                       startTimeVector) == false)
        return false;
    }

    return true; // all the conditions known to return false failed
  }

  bool WhetherJobStartEarlier(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                              SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector)
  {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

    // Current analysis on job group is not safe, let's' see what we can do without it
    // if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
    //   return false;
    return WhetherJobStartEarlierHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector);
  }

  bool WhetherJobStartLaterHelper(JobCEC jobCurr, JobCEC jobChanged,
                                  std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector)
  {
    if (std::abs(GetFinishTime(jobCurr, startTimeVector, tasksInfo) - GetDeadline(jobCurr, tasksInfo)) < 1e-3)
      return false;
    else if (jobCurr == jobChanged)
    {
      return true;
    }

    std::vector<JobCEC> followAdjacentJobs =
        FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    if (followAdjacentJobs.size() == 0) // this should never happen in case of LP order scheduler, and this
                                        // function should not be used with simple order scheduler
      return false;

    for (auto &jobFollow : followAdjacentJobs)
    {
      if (WhetherJobStartLaterHelper(jobFollow, jobChanged, jobGroupMap, jobOrder, tasksInfo,
                                     startTimeVector) == false)
        return false;
    }

    return true;
  }

  // this function requires that the start time is already maximum in startTimeVector
  // this function cannot work with SimpleOrderScheduler
  bool WhetherJobStartLater(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                            SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector)
  {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

    // Current analysis on job group is not safe, let's' see what we can do without it
    // if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
    //   return false;
    // int countPath = 0;
    // std::unordered_set<JobCEC> pathRecord;
    // pathRecord.reserve(tasksInfo.length);

    return WhetherJobStartLaterHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector);
  }

  std::vector<JobCEC> ActiveJobs::GetJobs() const
  {
    std::vector<JobCEC> jobs;
    jobs.reserve(size());
    for (auto &jobA : activeJobs)
      jobs.push_back(jobA.job);
    return jobs;
  }

  CentralJobs FindCentralJobs(const LongestCAChain &longestChain,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
  {
    std::unordered_set<JobCEC> centralSourceJobRecord;
    centralSourceJobRecord.reserve(tasksInfo.length);
    std::unordered_set<JobCEC> centralSinkJobRecord;
    centralSinkJobRecord.reserve(tasksInfo.length);
    CentralJobs centralJobs(tasksInfo.length);

    for (uint i = 0; i < longestChain.size(); i++)
    {
      auto &chainCurr = longestChain.longestChains_[i];
      if (chainCurr.size() > 0)
      {
        JobCEC jobSource = chainCurr[0].GetJobWithinHyperPeriod(tasksInfo);
        if (centralSourceJobRecord.find(jobSource) == centralSourceJobRecord.end())
        {
          centralSourceJobRecord.insert(jobSource);
          centralJobs.backwardJobs.push_back(jobSource);
        }

        JobCEC jobSink = ((chainCurr.back())).GetJobWithinHyperPeriod(tasksInfo);
        if (centralSinkJobRecord.find(jobSink) == centralSinkJobRecord.end())
        {
          centralSinkJobRecord.insert(jobSink);
          centralJobs.forwardJobs.push_back(jobSink);
        }
      }
    }
    return centralJobs;
  }

  ActiveJobs FindActiveJobs(const CentralJobs &centralJobs, SFOrder &jobOrder,
                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector)
  {
    std::vector<ActiveJob> activeJobs;
    std::unordered_set<JobCEC> jobRecord;
    jobRecord.reserve(tasksInfo.length);
    for (uint i = 0; i < centralJobs.forwardJobs.size(); i++)
    {
      JobCEC jobCurr = centralJobs.forwardJobs[i];
      std::vector<JobCEC> forwardJobs = FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
      forwardJobs.push_back(jobCurr);
      for (uint j = 0; j < forwardJobs.size(); j++)
      {
        if (jobRecord.find(forwardJobs[j]) == jobRecord.end())
        {
          jobRecord.insert(forwardJobs[j]);
          activeJobs.push_back(ActiveJob(forwardJobs[j], true));
        }
      }
    }

    for (uint i = 0; i < centralJobs.backwardJobs.size(); i++)
    {
      JobCEC jobCurr = centralJobs.backwardJobs[i];
      std::vector<JobCEC> backwardJobs = FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
      backwardJobs.push_back(jobCurr);
      for (uint j = 0; j < backwardJobs.size(); j++)
      {
        if (jobRecord.find(backwardJobs[j]) == jobRecord.end())
        {
          jobRecord.insert(backwardJobs[j]);
          activeJobs.push_back(ActiveJob(backwardJobs[j], false));
        }
      }
    }
    return ActiveJobs(activeJobs, jobRecord);
  }

} // namespace OrderOptDAG_SPACE