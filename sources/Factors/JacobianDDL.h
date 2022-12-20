#pragma once

#include "sources/Factors/AugmentedJacobian.h"

namespace OrderOptDAG_SPACE {

inline void UpdateAugmentedJacobianDDL(AugmentedJacobian &augJacob, int rowIndex, int jobIndex,
                                       const JobCEC &jobCurr, const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian(rowIndex, jobIndex) = 1;
  augJacob.rhs(rowIndex) = GetDeadline(jobCurr, tasksInfo) - GetExecutionTime(jobCurr, tasksInfo);
}

AugmentedJacobianTriplet GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo);

inline void UpdateAugmentedJacobianActivationTime(AugmentedJacobian &augJacob, int rowIndex, int jobIndex,
                                                  const JobCEC &jobCurr,
                                                  const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian(rowIndex, jobIndex) = -1;
  augJacob.rhs(rowIndex) = GetActivationTime(jobCurr, tasksInfo) * -1;
}

AugmentedJacobianTriplet GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo);

} // namespace OrderOptDAG_SPACE