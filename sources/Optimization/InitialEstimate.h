
#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Utils/colormod.h"
#include "sources/Optimization/ScheduleSimulation.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{

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
    VectorDynamic GenerateInitial(DAG_Model &dagTasks,
                                  std::vector<LLint> &sizeOfVariables,
                                  int variableDimension, VectorDynamic initialUser = GenerateVectorDynamic(1));

}