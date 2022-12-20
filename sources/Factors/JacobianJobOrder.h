#pragma once

#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE {

// a map that returns the occurence index of each job instance
std::unordered_map<JobCEC, size_t> GetJobOrderMap(const SFOrder &jobOrder);

void UpdateAugmentedJacobianJobOrder(AugmentedJacobian &augJacob, TimeInstance &instPrev,
                                     TimeInstance &instCurr, int rowIndex, int globalIdPrev, int globalIdCurr,
                                     const TaskSetInfoDerived &tasksInfo);

AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                      const SFOrder &jobOrder);

AugmentedJacobian
GetJacobianJobOrderReduced(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                           const SFOrder &jobOrder, int chainIndex,
                           const std::unordered_map<JobCEC, std::vector<JobCEC>> &reactionChainMap);
} // namespace OrderOptDAG_SPACE