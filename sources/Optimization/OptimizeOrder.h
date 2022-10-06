#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"

namespace DAG_SPACE
{

    template <class SchedulingAlgorithm>
    struct IterationStatus
    {
        DAG_Model dagTasks_;
        JobOrderMultiCore jobOrder_;
        int processorNum_;
        VectorDynamic startTimeVector_;
        std::vector<RTDA> rtdaVec_;
        RTDA maxRtda_;
        double objVal_;
        bool schedulable_;

        IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const JobOrderMultiCore &jobOrder, int processorNum) : dagTasks_(dagTasks), jobOrder_(jobOrder), processorNum_(processorNum)
        {
            // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
            startTimeVector_ = SchedulingAlgorithm::Schedule(dagTasks, tasksInfo, processorNum_, jobOrder_);
            rtdaVec_ = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], startTimeVector_);
            maxRtda_ = GetMaxRTDA(rtdaVec_);
            objVal_ = ObjRTDA(maxRtda_);
            schedulable_ = SchedulabilityCheck(dagTasks, tasksInfo, startTimeVector_);
        }
    };

    template <class SchedulingAlgorithm>
    bool MakeProgress(IterationStatus<SchedulingAlgorithm> &statusPrev, IterationStatus<SchedulingAlgorithm> &statusCurr)
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

    template <class SchedulingAlgorithm>
    ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, int processorNum = coreNumberAva)
    {
        if (dagTasks.chains_.size() == 0)
            CoutWarning("No chain is provided for the given dag!");

        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
        if (debugMode == 1)
        {
            std::cout << "Initial schedule: " << std::endl;
            PrintSchedule(tasksInfo, initialSTV);
        }

        JobOrderMultiCore jobOrderRef(tasksInfo, initialSTV);
        IterationStatus<SchedulingAlgorithm> statusPrev(dagTasks, tasksInfo, jobOrderRef, processorNum);
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
                    JobOrderMultiCore jobOrderCurr = statusPrev.jobOrder_;
                    jobOrderCurr.ChangeJobOrder(i, j);
                    IterationStatus<SchedulingAlgorithm> statusCurr(dagTasks, tasksInfo, jobOrderCurr, processorNum);
                    if (MakeProgress<SchedulingAlgorithm>(statusPrev, statusCurr))
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

            // TODO: iterate possible permutations of JobOrderMultiCore
        }
        // TODO: optimize the final schedule to reduce RTDA

        // JobOrder jobOrderCurr = statusPrev.jobOrder_;
        // jobOrderCurr.ChangeJobOrder(5, 0);
        // IterationStatus<SchedulingAlgorithm> statusCurr(dagTasks, tasksInfo, jobOrderCurr);
        // PrintSchedule(tasksInfo, statusCurr.startTimeVector_, processorNum);
        // if (MakeProgress<SchedulingAlgorithm>(statusPrev, statusCurr))
        // {
        //     findNewUpdate = true;
        //     statusPrev = statusCurr;
        //     if (debugMode == 1)
        //     {
        //         std::cout << "Make progress!" << std::endl;
        //         PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
        //     }
        // }

        ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.maxRtda_};
        return scheduleRes;
    }

} // namespace DAG_SPACE
