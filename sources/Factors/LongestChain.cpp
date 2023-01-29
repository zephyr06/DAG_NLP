#include "sources/Factors/LongestChain.h"

namespace OrderOptDAG_SPACE {

// sourceJob is not used because it is equivalent as jobChain[0]
double GetReactionTime(const std::vector<JobCEC> &jobChain, const VectorDynamic &startTimeVector,
                       const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  if (jobChain.size() <= 1) {
    CoutError("Cannot analyze reaction time because jobChain is too short!");
  };
  return GetFinishTime(jobChain.back(), startTimeVector, tasksInfo) -
         GetStartTime(jobChain[0], startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>>
GetMaxReactionTimeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                         const VectorDynamic &startTimeVector,
                         const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::vector<std::vector<JobCEC>> longestChains;
  longestChains.reserve(10); // there should be no more than 10 longest chains
  double maxLength = -2;
  for (const auto &pair : react_chain_map) {
    // const JobCEC &jobCurr = pair.first;
    const std::vector<JobCEC> &jobChain = pair.second;
    double lengthChainCurr = GetReactionTime(jobChain, startTimeVector, tasksInfo);
    if (lengthChainCurr > maxLength) {
      longestChains.clear();
      longestChains.push_back(jobChain);
      maxLength = lengthChainCurr;
    } else if (lengthChainCurr == maxLength) {
      longestChains.push_back(jobChain);
    }
  }
  return longestChains;
}

double GetDataAge(const std::vector<JobCEC> &jobChain, const std::vector<JobCEC> &prevJobChain,
                  const VectorDynamic &startTimeVector,
                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  if (jobChain.size() <= 1) {
    CoutError("Cannot analyze reaction time because jobChain is too short!");
  };

  JobCEC sinkJob = jobChain.back();
  sinkJob.jobId--;
  JobCEC sourceJob = jobChain[0];
  sourceJob.jobId--;
  if (sinkJob.jobId < 0 || sourceJob.jobId < 0 || prevJobChain.back() == jobChain.back())
    return -1;
  return GetFinishTime(sinkJob, startTimeVector, tasksInfo) -
         GetStartTime(sourceJob, startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>>
GetMaxDataAgeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                    const VectorDynamic &startTimeVector,
                    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::vector<std::vector<JobCEC>> longestChains;
  longestChains.reserve(10); // there should be no more than 10 longest chains
  double maxLength = -2;
  for (const auto &pair : react_chain_map) {
    const JobCEC &jobCurr = pair.first;
    const std::vector<JobCEC> &jobChain = pair.second;

    JobCEC jobPrev(jobCurr.taskId, jobCurr.jobId - 1);
    if (react_chain_map.find(jobPrev) == react_chain_map.end())
      continue;
    const std::vector<JobCEC> &prevJobChain = react_chain_map.at(jobPrev);
    double lengthChainCurr = GetDataAge(jobChain, prevJobChain, startTimeVector, tasksInfo);
    if (lengthChainCurr > maxLength) {
      longestChains.clear();
      longestChains.push_back(jobChain);
      maxLength = lengthChainCurr;
    } else if (lengthChainCurr == maxLength) {
      longestChains.push_back(jobChain);
    }
  }
  return longestChains;
}

std::vector<std::vector<JobCEC>> LongestCAChain::FindLongestCAChain(const DAG_Model &dagTasks,
                                                                    const TaskSetInfoDerived &tasksInfo,
                                                                    SFOrder &jobOrder,
                                                                    const VectorDynamic &startTimeVector,
                                                                    int processorNum) {
  BeginTimer("FindLongestCAChain");
  std::vector<std::vector<JobCEC>> longestChains;
  longestChains.reserve(dagTasks.chains_.size() * 3); // 3x is usually not necessary, though

  std::unordered_set<JobCEC> sourceJobRecords;

  for (uint i = 0; i < dagTasks.chains_.size(); i++) {
    auto react_chain_map =
        GetReactionChainMap(dagTasks, tasksInfo, jobOrder, processorNum, dagTasks.chains_[i], i);
    auto chains = GetMaxReactionTimeChains(react_chain_map, startTimeVector, tasksInfo);
    auto chains2 = GetMaxDataAgeChains(react_chain_map, startTimeVector, tasksInfo);
    chains.insert(chains.end(), chains2.begin(), chains2.end());
    for (uint i = 0; i < chains.size(); i++) {
      if (sourceJobRecords.find(chains[i][0]) == sourceJobRecords.end()) {
        longestChains.push_back(chains[i]);
        sourceJobRecords.insert(chains[i][0]);
      }
    }
  }
  EndTimer("FindLongestCAChain");
  return longestChains;
}

int FindSiblingJobIndex(const JobCEC &job, const std::vector<JobCEC> &jobChainCurr) {
  for (uint i = 0; i < jobChainCurr.size(); i++) {
    if (job.taskId == jobChainCurr[i].taskId) {
      return i;
    }
  }
  // CoutError("Didn't find sibling job in FindSiblingJobIndex");
  return -1;
}
// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove job, and then insert its start/finish instances at
// startP/finishP
// If you want to be safer, return more 'true'
bool WhetherJobBreakChain(const JobCEC &job, LLint startP, LLint finishP,
                          const LongestCAChain &longestJobChains, const DAG_Model &dagTasks,
                          SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo) {
  BeginTimer("WhetherJobBreakChain");
  for (auto &taskChainCurr : dagTasks.chains_) {
    auto itr = std::find(taskChainCurr.begin(), taskChainCurr.end(), job.taskId);
    if (itr != taskChainCurr.end()) {
      for (uint i = 0; i < longestJobChains.size(); i++) {
        const std::vector<JobCEC> &jobChainCurr = longestJobChains[i];
        int siblingJobIndex = FindSiblingJobIndex(job, jobChainCurr);
        if (siblingJobIndex == -1)
          continue;
        JobCEC sibJob = jobChainCurr[siblingJobIndex];
        if (sibJob == job) // this may not be necessary, but is a safe solution
        {
          EndTimer("WhetherJobBreakChain");
          return true;
        }

        if (siblingJobIndex == 0) { // new source job may initiate a different cause-effect chain
          JobCEC afterSibJob =
              jobChainCurr[siblingJobIndex + 1]; // assume the length of the chain is longer than 1
          if (sibJob.jobId < job.jobId && finishP < jobOrder.GetJobStartInstancePosition(afterSibJob)) {
            EndTimer("WhetherJobBreakChain");
            return true;
          }
        } else { // the job is not a source task's job
          if (sibJob.jobId < job.jobId)
            continue; // the job cannot react earlier than sibJob, and so cannot change reaction relationship
          JobCEC sibJobImmediateSourceJob = jobChainCurr[siblingJobIndex - 1];
          LLint sibImmeSourJobFinish = jobOrder.GetJobFinishInstancePosition(sibJobImmediateSourceJob);
          if (jobOrder.GetJobStartInstancePosition(job) < sibImmeSourJobFinish)
            sibImmeSourJobFinish--;
          if (jobOrder.GetJobFinishInstancePosition(job) < sibImmeSourJobFinish)
            sibImmeSourJobFinish--;
          if (sibImmeSourJobFinish <= startP && job.jobId < sibJob.jobId) {
            EndTimer("WhetherJobBreakChain");
            return true;
          }
        }
      }
    }
  }
  EndTimer("WhetherJobBreakChain");
  return false;
}
std::unordered_map<JobCEC, int> ExtractIndependentJobGroups(const SFOrder &jobOrder,
                                                            const TaskSetInfoDerived &tasksInfo) {
  BeginTimer("ExtractIndependentJobGroups");
  std::unordered_map<JobCEC, int> jobGroupMap;
  int jobGroupIndex = 0;
  jobGroupMap.insert({jobOrder.at(0).job, jobGroupIndex});
  for (uint i = 1; i < jobOrder.size(); i++) {
    const TimeInstance &instCurr = jobOrder.at(i);
    if (instCurr.type == 's') {
      auto a = jobOrder.at(i - 1).GetRangeMax(tasksInfo);
      auto b = instCurr.GetRangeMin(tasksInfo);
      if (a <= b) {
        jobGroupIndex++;
      }
      jobGroupMap.insert({instCurr.job, jobGroupIndex});
    } else {
      ;
    }
  }
  EndTimer("ExtractIndependentJobGroups");
  return jobGroupMap;
}

} // namespace OrderOptDAG_SPACE