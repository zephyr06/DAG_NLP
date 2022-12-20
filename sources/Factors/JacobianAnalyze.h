
#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Factors/JacobianCauseEffectChain.h"
#include "sources/Factors/JacobianDBF.h"
#include "sources/Factors/JacobianDDL.h"
#include "sources/Factors/JacobianJobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE {

AugmentedJacobian GetDAGJacobianOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                    SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                    int processorNum, bool lessJobOrderConstraints = false);

std::unordered_map<JobCEC, size_t> GetJobOrderMap(const SFOrder &jobOrder);

// This function requires more consideration
// order of AugmentedJacobian follows instanceOrder in jobOrder
// The columns of each Jacobian matrix follows instanceOrder in jobOrder
// TODO: clean code, refactor function
// return: all the Jacobian matrices, columns and rows are re-ordered following jobs' dispatch order
std::vector<AugmentedJacobian> GetVariableBlocks(const DAG_Model &dagTasks,
                                                 const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder,
                                                 const std::vector<uint> processorJobVec, int processorNum);

// Input is the original coefficient vector c for LP without "band" re-ordering trick
VectorDynamic ReOrderLPObj(const VectorDynamic &c, const SFOrder &jobOrder,
                           const TaskSetInfoDerived &tasksInfo);

inline AugmentedJacobian GetDAGJacobianOrdered(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                               const SFOrder &jobOrder,
                                               const std::vector<uint> processorJobVec, int processorNum) {
  std::vector<AugmentedJacobian> augJacobs =
      GetVariableBlocks(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
  return MergeAugJacobian(augJacobs);
}

AugmentedJacobian
GetJacobianJobOrderReduced(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                           const SFOrder &jobOrder, int chainIndex,
                           const std::unordered_map<JobCEC, std::vector<JobCEC>> &reactionChainMap);
} // namespace OrderOptDAG_SPACE