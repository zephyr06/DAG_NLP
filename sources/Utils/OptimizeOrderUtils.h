#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"

namespace DAG_SPACE
{

    struct ScheduleResult
    {
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        RTDA rtda_;
        double obj_;

        ScheduleResult() {}
        ScheduleResult(JobOrder jobOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       RTDA rtda) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), rtda_(rtda)
        {
            obj_ = ObjRTDA(rtda_);
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
        return 0 == graph.error(initialEstimateFG);
    }

    // this function seems unnecessary because list scheduling should return a solution without DBF error
    bool CheckDBFConstraint(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        for (auto itr = tasksInfo.processorTaskSet.begin(); itr != tasksInfo.processorTaskSet.end(); itr++)
        {
            if (DbfIntervalOverlapError(startTimeVector, itr->first,
                                        tasksInfo.processorTaskSet, tasksInfo.tasks, tasksInfo.sizeOfVariables) > 0)
                return false;
        }
        return true;
    }

    bool SchedulabilityCheck(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        return CheckDDLConstraint(dagTasks, tasksInfo, startTimeVector) && CheckDBFConstraint(dagTasks, tasksInfo, startTimeVector);
    }

    ScheduleResult ScheduleDAGLS_LFT(DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFT(dagTasks, tasksInfo);
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