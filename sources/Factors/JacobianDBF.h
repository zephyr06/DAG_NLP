#pragma once

#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE {

inline void UpdateAugmentedJacobianDBF(AugmentedJacobian &augJacob, int rowIndex, int globalIdPrev,
                                       int globalIdCurr, const JobCEC &jobPrev,
                                       const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian(rowIndex, globalIdPrev) = 1;
  augJacob.jacobian(rowIndex, globalIdCurr) = -1;
  augJacob.rhs(rowIndex) = -1 * GetExecutionTime(jobPrev, tasksInfo);
}
inline void UpdateAugmentedJacobianTripletDBF(AugmentedJacobianTriplet &augJacob, int rowIndex,
                                              int globalIdPrev, int globalIdCurr, const JobCEC &jobPrev,
                                              const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian.push_back(EigenTriplet(rowIndex, globalIdPrev, 1));
  augJacob.jacobian.push_back(EigenTriplet(rowIndex, globalIdCurr, -1));
  augJacob.rhs(rowIndex) = -1 * GetExecutionTime(jobPrev, tasksInfo);
}

std::vector<std::vector<JobCEC>>
SortJobsEachProcessor(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder,
                      const std::vector<uint> processorJobVec, int processorNum);

AugmentedJacobianTriplet GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                        const SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                        int processorNum);

} // namespace OrderOptDAG_SPACE