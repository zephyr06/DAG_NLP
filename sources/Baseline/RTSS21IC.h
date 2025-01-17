#pragma once

#include "sources/Baseline/OptimizeWithElimination.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Utils/OptimizeOrderUtils.h"

namespace OrderOptDAG_SPACE
{
    // TODO: make this work with fix-ed processor assignment
    // TODO: add a scheduler with fixed processor assignment?
    ScheduleResult ScheduleRTSS21IC(DAG_Model &dagTasks, double sfTolerance, double freshTol)
    {
        RTSS21IC_NLP::sensorFusionTolerance = sfTolerance;
        RTSS21IC_NLP::freshTol = freshTol;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        // VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, GlobalVariablesDAGOpt::coreNumberAva, std::nullopt, processorIdVec);

        VectorDynamic initial = SimulateFixedPrioritySched(dagTasks, tasksInfo);
        // std::vector<uint> processorIdVec;
        // VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, GlobalVariablesDAGOpt::coreNumberAva, processorIdVec);
        // RTSS21IC_NLP::processorIdVecGlobal = processorIdVec;
        RTSS21IC_NLP::processorNumGlobal = GlobalVariablesDAGOpt::coreNumberAva;
        auto sth = RTSS21IC_NLP::DAG_SPACE::OptimizeScheduling(dagTasks, initial);
        ScheduleResult res;
        res.schedulable_ = sth.optimizeError < 1e-1;
        res.startTimeVector_ = sth.optimizeVariable;

        return res;
    }

    bool ExamBasicFeasibilityRTSS21IC(DAG_Model &dagTasks)
    {
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        VectorDynamic initial = SimulateFixedPrioritySched(dagTasks, tasksInfo);

        return ExamDDL_Feasibility(dagTasks, tasksInfo, initial);
    }

}