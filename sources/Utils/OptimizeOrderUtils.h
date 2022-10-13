#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/JobOrder.h"

namespace DAG_SPACE
{

    struct ScheduleResult
    {
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        RTDA rtda_;
        double obj_;
        double timeTaken_;
        std::vector<uint> processorJobVec_;

        ScheduleResult() {}
        ScheduleResult(JobOrder jobOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       RTDA rtda) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), rtda_(rtda)
        {
            obj_ = ObjRTDA(rtda_);
            timeTaken_ = 0;
            processorJobVec_.clear();
        }
    };

    void PrintResultAnalyzation(ScheduleResult &scheduleResult, DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        std::cout << Color::blue;
        std::cout << "Schedulable after optimization? " << scheduleResult.schedulable_ << std::endl;
        RTDA resAfterOpt = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], scheduleResult.startTimeVector_);
        resAfterOpt.print();
        std::cout << Color::def << std::endl;
        std::cout << "Schedule: " << std::endl;
        PrintSchedule(tasksInfo, scheduleResult.startTimeVector_);
    }

    bool CheckDDLConstraint(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        gtsam::NonlinearFactorGraph graph;
        AddDDL_Factor(graph, tasksInfo);
        gtsam::Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        double err = graph.error(initialEstimateFG);
        return 0 == err;
    }

    bool SchedulabilityCheck(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        return CheckDDLConstraint(dagTasks, tasksInfo, startTimeVector);
    }

    ScheduleResult ScheduleDAGLS_LFT(DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, coreNumberAva);
        JobOrder jobOrderRef(tasksInfo, initialSTV);
        RTDA rtda = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], initialSTV);

        if (SchedulabilityCheck(dagTasks, tasksInfo, initialSTV))
        {
            ScheduleResult res{
                jobOrderRef,
                initialSTV,
                true, rtda};
            return res;
        }

        else
            return ScheduleResult{jobOrderRef, initialSTV, false, rtda};
    }
} // namespace DAG_SPACE