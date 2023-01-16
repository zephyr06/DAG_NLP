
#pragma once
#include "unordered_map"

#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Utils/colormod.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// processorIdVec is actually an argument that serves as output
template <typename ObjectiveFunctionBase>
VectorDynamic SelectInitialFromPool(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                    const ScheduleOptions &scheduleOptions,
                                    boost::optional<std::vector<uint> &> processorIdVec = boost::none) {
  BeginTimer("SelectInitialFromPool");
  std::vector<uint> processorJobVec;
  VectorDynamic stvBest =
      ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
  double objMin = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, stvBest, scheduleOptions);
  int candidateNum = scheduleOptions.selectInitialFromPoolCandidate_;
  if (candidateNum != 0) {
    for (double modifyPower = 0; modifyPower < 5; modifyPower += 5.0 / candidateNum) {
      double modifyCoeff = std::pow(10, modifyPower);
      std::vector<uint> processorJobVecCurr;
      BeginTimer("SelectInitialFromPool_list_scheduling");
      VectorDynamic stvCurr = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_,
                                                  processorJobVecCurr, modifyCoeff);
      EndTimer("SelectInitialFromPool_list_scheduling");
      double objCurr = ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, stvCurr, scheduleOptions);
      if (objCurr < objMin && ExamBasic_Feasibility(dagTasks, tasksInfo, stvCurr, processorJobVecCurr,
                                                    scheduleOptions.processorNum_)) {
        objMin = objCurr;
        stvBest = stvCurr;
        processorJobVec = processorJobVecCurr;
      }
    }
  }
  if (processorIdVec != boost::none)
    *processorIdVec = processorJobVec;
  EndTimer("SelectInitialFromPool");
  return stvBest;
}
} // namespace OptimizeSF

gtsam::Values GenerateInitialFG(const VectorDynamic &startTimeVector, const TaskSetInfoDerived &tasksInfo);

/**
 * @brief Generate initial estimate based on provided options'
 * param: initializeMethod: global parameter passed implicitly
 *
 * @param dagTasks
 * @param sizeOfVariables
 * @param variableDimension
 * @return VectorDynamic
 */
VectorDynamic GenerateInitial(DAG_Model &dagTasks, std::vector<LLint> &sizeOfVariables, int variableDimension,
                              VectorDynamic initialUser = GenerateVectorDynamic(1));

} // namespace OrderOptDAG_SPACE