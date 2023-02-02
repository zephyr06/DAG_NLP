
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
      if (WhetherImmediateAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector)) {
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
bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
                                  std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector,
                                  std::unordered_set<JobCEC> &pathRecord, int &countPath) {
  if (countPath > 10)
    return true;
  if (jobCurr == jobChanged) {
    countPath++;
    return true;
  } else if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) -
                      GetActivationTime(jobCurr, tasksInfo)) < 1e-3)
    return false;

  std::vector<JobCEC> prevAdjacentJobs =
      FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (prevAdjacentJobs.size() == 0)
    return false;

  bool existUnvisitedIAJ = false; // IAJ means immediate adjacent jobs
  for (auto &jobPrev : prevAdjacentJobs) {
    if (pathRecord.find(jobPrev) != pathRecord.end())
      continue; // avoid adding repeated jobs
    else {
      pathRecord.insert(jobPrev);
      existUnvisitedIAJ = true;
      if (WhetherJobStartEarlierHelper(jobPrev, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector,
                                       pathRecord, countPath) == false)
        return false;
      pathRecord.erase(jobPrev);
    }
  }
  if (!existUnvisitedIAJ)
    return false;

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
  int countPath = 0;
  std::unordered_set<JobCEC> pathRecord;
  pathRecord.reserve(tasksInfo.length);

  return WhetherJobStartEarlierHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector,
                                      pathRecord, countPath);
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
      if (WhetherImmediateAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector)) {
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
                                const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &pathRecord,
                                int &countPath) {
  if (countPath > 10)
    return true;
  if (jobCurr == jobChanged) {
    countPath++;
    return true;
  } else if (std::abs(GetFinishTime(jobCurr, startTimeVector, tasksInfo) - GetDeadline(jobCurr, tasksInfo)) <
             1e-3)
    return false;

  std::vector<JobCEC> followAdjacentJobs =
      FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (followAdjacentJobs.size() == 0) // this should never happen in case of LP order scheduler, and this
                                      // function should not be used with simple order scheduler
    return false;

  bool existUnvisitedIAJ = false; // IAJ means immediate adjacent jobs
  for (auto &jobFollow : followAdjacentJobs) {
    if (pathRecord.find(jobFollow) != pathRecord.end())
      continue; // avoid adding repeated jobs
    else {
      pathRecord.insert(jobFollow);
      existUnvisitedIAJ = true;
      if (WhetherJobStartLaterHelper(jobFollow, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector,
                                     pathRecord, countPath) == false)
        return false;
      pathRecord.erase(jobFollow);
    }
  }
  if (!existUnvisitedIAJ)
    return false;
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
  int countPath = 0;
  std::unordered_set<JobCEC> pathRecord;
  pathRecord.reserve(tasksInfo.length);

  return WhetherJobStartLaterHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector,
                                    pathRecord, countPath);
}

} // namespace OrderOptDAG_SPACE