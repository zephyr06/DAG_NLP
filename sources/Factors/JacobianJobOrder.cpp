#include "sources/Factors/JacobianJobOrder.h"

namespace OrderOptDAG_SPACE {

void UpdateAugmentedJacobianJobOrder(AugmentedJacobian &augJacob, TimeInstance &instPrev,
                                     TimeInstance &instCurr, int rowIndex, int globalIdPrev, int globalIdCurr,
                                     const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian.row(rowIndex).setZero();
  augJacob.jacobian(rowIndex, globalIdCurr) = -1;
  augJacob.jacobian(rowIndex, globalIdPrev) = 1;
  if (instPrev.type == 's') {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = 0;
    } else // instCurr.type == 'f'
    {
      augJacob.rhs(rowIndex) = GetExecutionTime(instCurr.job, tasksInfo);
    }
  } else // instPrev.type == 'f'
  {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
    } else // type == 'f'
    {
      augJacob.rhs(rowIndex) =
          -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
    }
  }
}

// Bug found: it can't correctly distinguish prev and next instances in cases such as (1,0,s) preceeds
// (0,0,f), but we'll probably not use this function anymore
AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                      const SFOrder &jobOrder) {
  AugmentedJacobian augJacob;
  augJacob.jacobian.conservativeResize(tasksInfo.length * 2 - 1, tasksInfo.length);
  augJacob.jacobian.setZero();
  augJacob.rhs.conservativeResize(tasksInfo.length * 2 - 1, 1);
  augJacob.rhs.setZero();

  int count = 0;
  const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
  for (uint i = 1; i < instanceOrder.size(); i++) {
    auto instCurr = instanceOrder[i];
    auto instPrev = instanceOrder[i - 1];
    if (instPrev.job == instCurr.job)
      continue;
    int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo);
    int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo);
    UpdateAugmentedJacobianJobOrder(augJacob, instPrev, instCurr, count, globalIdPrev, globalIdCurr,
                                    tasksInfo);
    count++;
  }

  augJacob.jacobian.conservativeResize(count, tasksInfo.length);
  augJacob.rhs.conservativeResize(count, 1);
  return augJacob;
}
} // namespace OrderOptDAG_SPACE