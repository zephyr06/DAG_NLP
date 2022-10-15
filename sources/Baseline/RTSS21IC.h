#pragma once

#include "sources/Optimization/OptimizeWithElimination.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Utils/OptimizeOrderUtils.h"

namespace OrderOptDAG_SPACE
{

    ScheduleResult ScheduleRTSS21IC(DAG_Model &dagTasks, double sfTolerance, double freshTol)
    {
        RTSS21IC_NLP::sensorFusionTolerance = sfTolerance;
        RTSS21IC_NLP::FreshTol = freshTol;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        std::vector<uint> processorIdVec;
        VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, coreNumberAva, std::nullopt, processorIdVec);
        auto sth = RTSS21IC_NLP::DAG_SPACE::OptimizeScheduling(dagTasks, initial);
        ScheduleResult res;
        res.schedulable_ = sth.optimizeError < 1e-1;

        return res;
    }

}