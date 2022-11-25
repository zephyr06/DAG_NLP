
#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Utils/colormod.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/ScheduleOptions.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        template <typename ObjectiveFunctionBase>
        VectorDynamic SelectInitialFromPool(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const ScheduleOptions &scheduleOptions)
        {
            VectorDynamic stvBest = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
            double objMin = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, stvBest, scheduleOptions);
            std::vector<uint> processorJobVec;
            for (double modifyCoeff = 0; modifyCoeff < 1000; modifyCoeff += 10)
            {
                VectorDynamic stvCurr = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec, modifyCoeff);
                double objCurr = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, stvCurr, scheduleOptions);
                if (objCurr < objMin)
                {
                    objMin = objCurr;
                    stvBest = stvCurr;
                }
            }
            return stvBest;
        }
    }

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