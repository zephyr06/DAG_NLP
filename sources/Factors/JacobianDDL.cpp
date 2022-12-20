#include "sources/Factors/JacobianDDL.h"

namespace OrderOptDAG_SPACE {

inline void UpdateAugmentedJacobianTripletDDL(AugmentedJacobianTriplet &augJacob, int rowIndex, int jobIndex,
                                              const JobCEC &jobCurr, const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian.push_back(EigenTriplet(rowIndex, jobIndex, 1));
  augJacob.rhs(rowIndex) = GetDeadline(jobCurr, tasksInfo) - GetExecutionTime(jobCurr, tasksInfo);
}

AugmentedJacobianTriplet GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo) {
  AugmentedJacobianTriplet augJacob;
  augJacob.jacobian.reserve(tasksInfo.length);
  // augJacob.jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);
  augJacob.rhs = GenerateVectorDynamic(tasksInfo.length);

  int count = 0;
  for (uint i = 0; i < dagTasks.tasks.size(); i++) {
    for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++) {
      JobCEC jobCurr((int)i, j);
      // UpdateAugmentedJacobianDDL(augJacob, count, count, jobCurr, tasksInfo);
      UpdateAugmentedJacobianTripletDDL(augJacob, count, count, jobCurr, tasksInfo);
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