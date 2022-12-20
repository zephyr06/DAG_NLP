#pragma once

#include "sources/Factors/AugmentedJacobian.h"
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE {

void UpdateAugmentedJacobianJobOrder(AugmentedJacobian &augJacob, TimeInstance &instPrev,
                                     TimeInstance &instCurr, int rowIndex, int globalIdPrev, int globalIdCurr,
                                     const TaskSetInfoDerived &tasksInfo);

AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                      const SFOrder &jobOrder);
} // namespace OrderOptDAG_SPACE