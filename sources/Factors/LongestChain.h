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

  std::vector<JobCEC> operator[](size_t index) const { return longestChains_[index]; }
  // std::vector<JobCEC> at(size_t index) const { return longestChains_[index]; }
  // data members
  std::vector<std::vector<JobCEC>> longestChains_;
};
} // namespace OrderOptDAG_SPACE