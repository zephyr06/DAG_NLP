#include "sources/Factors/JacobianDDL.h"

namespace OrderOptDAG_SPACE {

AugmentedJacobian GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo) {
  AugmentedJacobian augJacob;
  augJacob.jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);
  augJacob.rhs = GenerateVectorDynamic(tasksInfo.length);

  int count = 0;
  for (uint i = 0; i < dagTasks.tasks.size(); i++) {
    for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++) {
      JobCEC jobCurr((int)i, j);
      UpdateAugmentedJacobianDDL(augJacob, count, count, jobCurr, tasksInfo);
      count++;
    }
  }
  return augJacob;
}

AugmentedJacobian GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo) {
  AugmentedJacobian augJacob;
  augJacob.jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);
  augJacob.rhs = GenerateVectorDynamic(tasksInfo.length);

  int count = 0;
  for (uint i = 0; i < dagTasks.tasks.size(); i++) {
    for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++) {
      JobCEC jobCurr((int)i, j);
      UpdateAugmentedJacobianActivationTime(augJacob, count, count, jobCurr, tasksInfo);
      count++;
    }
  }
  return augJacob;
}
} // namespace OrderOptDAG_SPACE