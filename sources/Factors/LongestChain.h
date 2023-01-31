#pragma once
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

// sourceJob is not used because it is equivalent as jobChain[0]
double GetReactionTime(const std::vector<JobCEC> &jobChain, const VectorDynamic &startTimeVector,
                       const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

std::vector<std::vector<JobCEC>>
GetMaxReactionTimeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                         const VectorDynamic &startTimeVector,
                         const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

double GetDataAge(const std::vector<JobCEC> &jobChain, const std::vector<JobCEC> &prevJobChain,
                  const VectorDynamic &startTimeVector,
                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

std::vector<std::vector<JobCEC>>
GetMaxDataAgeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                    const VectorDynamic &startTimeVector,
                    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

class LongestCAChain {
public:
  LongestCAChain() {}
  LongestCAChain(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                 const VectorDynamic &startTimeVector, int processorNum)
      : longestChains_(FindLongestCAChain(dagTasks, tasksInfo, jobOrder, startTimeVector, processorNum)) {}

  std::vector<std::vector<JobCEC>> FindLongestCAChain(const DAG_Model &dagTasks,
                                                      const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                                                      const VectorDynamic &startTimeVector, int processorNum);

  void FindLongestCAChain() {}

  size_t size() const { return longestChains_.size(); }
  // inline begin() const { return longestChains_[0]; }
  // inline back() const { return longestChains_[size() - 1]; }

  std::vector<JobCEC> operator[](size_t index) const { return longestChains_[index]; }
  // std::vector<JobCEC> at(size_t index) const { return longestChains_[index]; }
  // data members
  std::vector<std::vector<JobCEC>> longestChains_;
};

int FindSiblingJobIndex(const JobCEC &job, const std::vector<JobCEC> &jobChainCurr);
// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove job, and then insert its start/finish instances at
// startP/finishP
// If you want to be safer, return more 'true'
bool WhetherJobBreakChain(const JobCEC &job, LLint startP, LLint finishP,
                          const LongestCAChain &longestJobChains, const DAG_Model &dagTasks,
                          SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo);

std::unordered_map<JobCEC, int> ExtractIndependentJobGroups(const SFOrder &jobOrder,
                                                            const TaskSetInfoDerived &tasksInfo);

inline bool WhetherInfluenceJobSimple(const JobCEC &jobCurr, const JobCEC &jobChanged,
                                      std::unordered_map<JobCEC, int> &jobGroupMap) {
  return (jobGroupMap[jobCurr] == jobGroupMap[jobChanged]);
}

bool WhetherInfluenceJobSource( JobCEC jobCurr, const JobCEC &jobChanged,
                               std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder, LLint startP,
                               LLint finishP, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);
bool WhetherInfluenceJobSink( JobCEC jobCurr, const JobCEC &jobChanged,
                             std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder, LLint startP,
                             LLint finishP, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);
} // namespace OrderOptDAG_SPACE