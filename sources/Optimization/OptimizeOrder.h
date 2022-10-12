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
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"

namespace DAG_SPACE
{
    bool ExamFeasibility(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {
        if (processorNum <= 0)
            return false;
        std::vector<std::vector<Interval>> jobsPerProcessor(processorNum);
        int index = 0;
        for (uint i = 0; i < dagTasks.tasks.size(); i++)
        {
            for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC job(i, j);
                Interval v(GetStartTime(job, startTimeVector, tasksInfo), tasksInfo.tasks[i].executionTime);
                if (v.start < tasksInfo.tasks[i].period * j)
                    return false;
                else if (v.start + v.length > tasksInfo.tasks[i].period * j + tasksInfo.tasks[i].deadline)
                    return false;
                if (processorJobVec[index] >= jobsPerProcessor.size())
                    return false;
                jobsPerProcessor[processorJobVec[index]].push_back(v);
                index++;
            }
        }
        for (int i = 0; i < processorNum; i++)
        {
            if (IntervalOverlapError(jobsPerProcessor[i]) > 0)
                return false;
        }
        return true;
    }

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
        else if (overallObjCurr < overallObjPrev && ((double)rand() / (RAND_MAX)) < RandomAccept)
            return true;
        return false;
    }

    bool WhetherSkipSwitch(const TaskSetInfoDerived &tasksInfo, const JobCEC &j1, const JobCEC &j2)
    {
        if (j1 == j2)
            return true;

        // Such switch always returns infeasible results
        if (j1.taskId == j2.taskId)
            return true;
        Interval v1(tasksInfo.tasks[j1.taskId].period * j1.jobId, tasksInfo.tasks[j1.taskId].period);
        Interval v2(tasksInfo.tasks[j2.taskId].period * j2.jobId, tasksInfo.tasks[j2.taskId].period);
        if (v2.start > v1.start + v1.length)
            return true;
        return false;
    }

    template <class SchedulingAlgorithm>
    ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, int processorNum = coreNumberAva)
    {
        // srand(RandomDrawWeightMaxLoop);
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

        auto ExamAndApplyUpdate = [&](JobOrderMultiCore jobOrderCurr)
        {
            IterationStatus<SchedulingAlgorithm> statusCurr(dagTasks, tasksInfo, jobOrderCurr, processorNum);

            // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
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
        };

        while (findNewUpdate)
        {
            findNewUpdate = false;
            for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
            {
                for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                {
                    JobOrderMultiCore jobOrderCurr = statusPrev.jobOrder_;
                    if (WhetherSkipSwitch(tasksInfo, jobOrderCurr[i], jobOrderCurr[j]))
                        continue;
                    jobOrderCurr.ChangeJobStartOrder(i, j);
                    ExamAndApplyUpdate(jobOrderCurr);
                }
            }
            if (processorNum > 1)
            {
                // Initialize it with a pair of jobs
                if (statusPrev.jobOrder_.sizeSerial() == 0)
                {
                    for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
                    {
                        for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                        {
                            JobOrderMultiCore jobOrderCurr = statusPrev.jobOrder_;
                            if (WhetherSkipSwitch(tasksInfo, jobOrderCurr[i], jobOrderCurr[j]))
                                continue;
                            jobOrderCurr.insertNP(i);
                            jobOrderCurr.insertNP(j);
                            jobOrderCurr.ChangeJobOrder(i, j);
                            ExamAndApplyUpdate(jobOrderCurr); // update outside variables
                        }
                    }
                }
                else
                {
                    for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
                    {
                        JobOrderMultiCore jobOrderCurr = statusPrev.jobOrder_;
                        jobOrderCurr.jobOrderSerial_[i] = !jobOrderCurr.jobOrderSerial_[i];
                        ExamAndApplyUpdate(jobOrderCurr); // update outside variables
                    }
                }
            }
        }

        ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.maxRtda_};
        if (doScheduleOptimization)
        {
            auto no_thing = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, statusPrev.jobOrder_, scheduleRes.processorJobVec_);

            ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
            ScheduleResult result_after_optimization;
            schedule_optimizer.Optimize(dagTasks, scheduleRes);
            result_after_optimization = schedule_optimizer.getOptimizedResult();
            return result_after_optimization;
        }

        if (!ExamFeasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum))
        {
            CoutError("Found one unschedulable case after optimization!");
        }
        return scheduleRes;
    }

} // namespace DAG_SPACE
