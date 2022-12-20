#pragma once

#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Factors/RTDA_Factor.h"

namespace OrderOptDAG_SPACE {
AugmentedJacobian GetJacobianCauseEffectChainOrg(const DAG_Model &dagTasks,
                                                 const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                                                 const std::vector<uint> processorJobVec, int processorNum,
                                                 const std::vector<int> &causeEffectChain, int chainIndex);

AugmentedJacobian MergeJacobianOfRTDAChains(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                            SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                            int processorNum);

} // namespace OrderOptDAG_SPACE