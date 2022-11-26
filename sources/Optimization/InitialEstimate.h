
#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Utils/colormod.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/OptimizeOrderUtils.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        // processorIdVec is actually an argument that serves as output
        template <typename ObjectiveFunctionBase>
        VectorDynamic SelectInitialFromPool(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const ScheduleOptions &scheduleOptions,
                                            boost::optional<std::vector<uint> &> processorIdVec = boost::none)
        {
            std::vector<uint> processorJobVec;
            VectorDynamic stvBest = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
            double objMin = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, stvBest, scheduleOptions);
            for (double modifyCoeff = 0; modifyCoeff < 1000; modifyCoeff += 10)
            {
                std::vector<uint> processorJobVecCurr;
                VectorDynamic stvCurr = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVecCurr, modifyCoeff);
                double objCurr = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, stvCurr, scheduleOptions);
                if (objCurr < objMin && ExamBasic_Feasibility(dagTasks, tasksInfo, stvCurr, processorJobVecCurr, scheduleOptions.processorNum_))
                {
                    objMin = objCurr;
                    stvBest = stvCurr;
                    processorJobVec = processorJobVecCurr;
                }
            }
            if (processorIdVec != boost::none)
                *processorIdVec = processorJobVec;
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