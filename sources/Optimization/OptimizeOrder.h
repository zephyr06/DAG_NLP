#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"

namespace DAG_SPACE
{
    struct ScheduleResult
    {
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        RTDA rtda_;

        ScheduleResult() {}
        ScheduleResult(JobOrder jobOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       RTDA rtda) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), rtda_(rtda) {}
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

    struct IterationStatus
    {
        DAG_Model dagTasks_;
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        std::vector<RTDA> rtdaVec_;
        RTDA maxRtda_;
        double objVal_;
        bool schedulable_;

        IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const JobOrder &jobOrder) : dagTasks_(dagTasks), jobOrder_(jobOrder)
        {
            startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
            rtdaVec_ = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], startTimeVector_);
            maxRtda_ = GetMaxRTDA(rtdaVec_);
            objVal_ = ObjRTDA(maxRtda_);
            schedulable_ = SchedulabilityCheck(dagTasks, tasksInfo, startTimeVector_);
        }
    };

    bool MakeProgress(IterationStatus &statusPrev, IterationStatus &statusCurr)
    {
        if (!statusCurr.schedulable_)
            return false;

        if (statusCurr.objVal_ < statusPrev.objVal_)
            return true;

        // whether average of chains decrease
        double overallObjPrev = 0;
        double overallObjCurr = 0;
        for (uint i = 0; i < statusPrev.rtdaVec_.size(); i++)
        {
            overallObjPrev += ObjRTDA(statusPrev.rtdaVec_[i]);
            overallObjCurr += ObjRTDA(statusCurr.rtdaVec_[i]);
        }
        if (overallObjCurr < overallObjPrev && statusPrev.objVal_ == statusCurr.objVal_)
            return true;

        return false;
    }

    ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFT(dagTasks, tasksInfo);
        if (debugMode == 1)
        {
            std::cout << "Initial schedule: " << std::endl;
            PrintSchedule(tasksInfo, initialSTV);
        }

        JobOrder jobOrderRef(tasksInfo, initialSTV);
        IterationStatus statusPrev(dagTasks, tasksInfo, jobOrderRef);
        if (!statusPrev.schedulable_)
        {
            CoutWarning("Initial schedule is not schedulable!!!");
        }

        bool findNewUpdate = true;
        while (findNewUpdate)
        {
            findNewUpdate = false;
            for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
            {
                for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                {
                    JobOrder jobOrderCurr = statusPrev.jobOrder_;
                    jobOrderCurr.ChangeJobOrder(i, j);
                    IterationStatus statusCurr(dagTasks, tasksInfo, jobOrderCurr);
                    if (MakeProgress(statusPrev, statusCurr))
                    {
                        findNewUpdate = true;
                        statusPrev = statusCurr;
                        if (debugMode == 1)
                        {
                            std::cout << "Make progress!" << std::endl;
                            PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                        }
                    }
                }
            }
        }
        // TODO: optimize the final schedule to reduce RTDA

        ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.maxRtda_};
        return scheduleRes;
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
