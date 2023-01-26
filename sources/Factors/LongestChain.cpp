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
  return longestChains;
}

} // namespace OrderOptDAG_SPACE