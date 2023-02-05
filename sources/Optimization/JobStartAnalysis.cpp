
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector, double tolerance) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
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
                                     double tolerance) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  }

  return false;
}

// previous adjacent job means the jobs whose finish time equals the start time of jobCurr
std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                           const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                           const VectorDynamic &startTimeVector) {
  std::vector<JobCEC> prevAdjacentJobs;
  prevAdjacentJobs.reserve(4 * 2); // actually, cannot be more than #core*2

  std::unordered_set<JobCEC> record;
  record.reserve(4 * 2);

  LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
  TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
  auto AddImmediateAdjacentInstance = [&](TimeInstance &instCurrJob, LLint jobInstIndex) {
    for (int i = jobInstIndex - 1; i >= 0; i--) {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateForwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder)) {
        if (record.find(instIte.job) == record.end()) {
          prevAdjacentJobs.push_back(instIte.job);
          record.insert(instIte.job);
        } else
          break;
      } else
        break;
    }
  };
  AddImmediateAdjacentInstance(instCurrJobStart, jobStartIndex);

  LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
  TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
  AddImmediateAdjacentInstance(instCurrJobFinish, jobFinishIndex);
  return prevAdjacentJobs;
}

// TODO: consider utilize startP and finishP
// Assumption: start time of jobCurr in startTimeVector cannot move earlier
bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
                                  std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector) {
  // ,
  // std::unordered_set<JobCEC> &pathRecord, int &countPath
  // if (countPath > 10)
  //   return true;
  if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) - GetActivationTime(jobCurr, tasksInfo)) <
      1e-3)
    return false;
  else if (jobCurr == jobChanged) {
    // countPath++;
    return true;
  }

  std::vector<JobCEC> prevAdjacentJobs =
      FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (prevAdjacentJobs.size() == 0)
    return false; // consider more, should be false under assumptions  of startTimeVector

  for (auto &jobPrev : prevAdjacentJobs) {
    if (WhetherJobStartEarlierHelper(jobPrev, jobChanged, jobGroupMap, jobOrder, tasksInfo,
                                     startTimeVector) == false)
      return false;
  }

  return true; // all the conditions known to return false failed
}

bool WhetherJobStartEarlier(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                            SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector) {
  jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
  jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

  // Current analysis on job group is not safe, let's' see what we can do without it
  // if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
  //   return false;
  return WhetherJobStartEarlierHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector);
}
bool WhetherImmediateBackwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                      const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                      const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                      double tolerance) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) >
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) >
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) >
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) >
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  }

  return false;
}

// 'backward' means finding the jobs whose index is larger than job, i.e., -->
std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                            const VectorDynamic &startTimeVector) {
  std::vector<JobCEC> followAdjacentJobs;
  followAdjacentJobs.reserve(4 * 2); // actually, cannot be more than #core*2

  std::unordered_set<JobCEC> record;
  record.reserve(4 * 2);

  LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
  TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
  auto AddImmediateAdjacentInstance = [&](TimeInstance &instCurrJob, LLint jobInstIndex) {
    for (uint i = jobInstIndex + 1; i < jobOrder.size(); i++) {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateBackwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder)) {
        if (record.find(instIte.job) == record.end()) {
          followAdjacentJobs.push_back(instIte.job);
          record.insert(instIte.job);
        } else
          break;
      } else
        break;
    }
  };
  AddImmediateAdjacentInstance(instCurrJobStart, jobStartIndex);

  LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
  TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
  AddImmediateAdjacentInstance(instCurrJobFinish, jobFinishIndex);
  return followAdjacentJobs;
}

bool WhetherJobStartLaterHelper(JobCEC jobCurr, JobCEC jobChanged,
                                std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                const VectorDynamic &startTimeVector) {
  if (std::abs(GetFinishTime(jobCurr, startTimeVector, tasksInfo) - GetDeadline(jobCurr, tasksInfo)) < 1e-3)
    return false;
  else if (jobCurr == jobChanged) {
    return true;
  }

  std::vector<JobCEC> followAdjacentJobs =
      FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (followAdjacentJobs.size() == 0) // this should never happen in case of LP order scheduler, and this
                                      // function should not be used with simple order scheduler
    return false;

  for (auto &jobFollow : followAdjacentJobs) {
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
                          const VectorDynamic &startTimeVector) {
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

CentralJobs FindCentralJobs(const LongestCAChain &longestChain,
                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::unordered_set<JobCEC> centralSourceJobRecord;
  centralSourceJobRecord.reserve(tasksInfo.length);
  std::unordered_set<JobCEC> centralSinkJobRecord;
  centralSinkJobRecord.reserve(tasksInfo.length);
  CentralJobs centralJobs(tasksInfo.length);

  for (uint i = 0; i < longestChain.size(); i++) {
    auto &chainCurr = longestChain.longestChains_[i];
    if (chainCurr.size() > 0) {
      JobCEC jobSource = chainCurr[0].GetJobWithinHyperPeriod(tasksInfo);
      if (centralSourceJobRecord.find(jobSource) == centralSourceJobRecord.end()) {
        centralSourceJobRecord.insert(jobSource);
        centralJobs.backwardJobs.push_back(jobSource);
      }

      JobCEC jobSink = ((chainCurr.back())).GetJobWithinHyperPeriod(tasksInfo);
      if (centralSinkJobRecord.find(jobSink) == centralSinkJobRecord.end()) {
        centralSinkJobRecord.insert(jobSink);
        centralJobs.forwardJobs.push_back(jobSink);
      }
    }
  }
  return centralJobs;
}

ActiveJobs FindActiveJobs(const CentralJobs &centralJobs, SFOrder &jobOrder,
                          const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                          const VectorDynamic &startTimeVector) {
  std::vector<JobCEC> activeJobs;
  std::unordered_set<JobCEC> jobRecord;
  jobRecord.reserve(tasksInfo.length);
  for (uint i = 0; i < centralJobs.forwardJobs.size(); i++) {
    JobCEC jobCurr = centralJobs.forwardJobs[i];
    std::vector<JobCEC> forwardJobs = FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    forwardJobs.push_back(jobCurr);
    for (uint j = 0; j < forwardJobs.size(); j++) {
      if (jobRecord.find(forwardJobs[j]) == jobRecord.end()) {
        jobRecord.insert(forwardJobs[j]);
        activeJobs.push_back(forwardJobs[j]);
      }
    }
  }

  for (uint i = 0; i < centralJobs.backwardJobs.size(); i++) {
    JobCEC jobCurr = centralJobs.backwardJobs[i];
    std::vector<JobCEC> backwardJobs = FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    backwardJobs.push_back(jobCurr);
    for (uint j = 0; j < backwardJobs.size(); j++) {
      if (jobRecord.find(backwardJobs[j]) == jobRecord.end()) {
        jobRecord.insert(backwardJobs[j]);
        activeJobs.push_back(backwardJobs[j]);
      }
    }
  }
  return ActiveJobs(activeJobs, jobRecord);
}

} // namespace OrderOptDAG_SPACE