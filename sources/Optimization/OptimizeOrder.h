#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Factors/SensorFusionFactor.h"

namespace OrderOptDAG_SPACE
{

    template <class SchedulingAlgorithm>
    struct IterationStatus
    {
        DAG_Model dagTasks_;
        JobOrderMultiCore jobOrder_;
        int processorNum_;
        VectorDynamic startTimeVector_;
        std::vector<uint> processorJobVec_;
        std::vector<std::vector<RTDA>> rtdaVec_; // for each chain
        std::vector<RTDA> maxRtda_;              // for each chain
        // double objVal_;
        bool schedulable_; // only basic schedulability
        VectorDynamic sfVec_;

        IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const JobOrderMultiCore &jobOrder, int processorNum) : dagTasks_(dagTasks), jobOrder_(jobOrder), processorNum_(processorNum)
        {
            // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
            processorJobVec_.clear();
            startTimeVector_ = SchedulingAlgorithm::Schedule(dagTasks, tasksInfo, processorNum_, jobOrder_, processorJobVec_);

            for (uint i = 0; i < dagTasks.chains_.size(); i++)
            {
                auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                rtdaVec_.push_back(rtdaVecTemp);
                maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
            }
            if (considerSensorFusion)
            {
                sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                // objVal_ += ObjSF(sfVec_);
            }
            schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_, sensorFusionTolerance, FreshTol);

            if (doScheduleOptimization)
            {
                ScheduleResult scheduleResBeforeOpt{jobOrder_, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};
                ScheduleResult resultAfterOptimization;
                ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
                scheduleOptimizer.Optimize(dagTasks, scheduleResBeforeOpt);
                resultAfterOptimization = scheduleOptimizer.getOptimizedResult();
                // if (debugMode && !resultAfterOptimization.schedulable_)
                // {
                //     std::cout << "Found one unschedulable case after optimization!\n";
                // }
                if (resultAfterOptimization.schedulable_)
                {
                    startTimeVector_ = resultAfterOptimization.startTimeVector_;
                    rtdaVec_.clear();
                    maxRtda_.clear();
                    for (uint i = 0; i < dagTasks.chains_.size(); i++)
                    {
                        auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                        rtdaVec_.push_back(rtdaVecTemp);
                        maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
                    }
                    if (considerSensorFusion)
                    {
                        sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                        // objVal_ += ObjSF(sfVec_);
                    }
                    schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_, sensorFusionTolerance, FreshTol);
                    jobOrder_ = JobOrderMultiCore(tasksInfo, startTimeVector_);
                }
            }
        }
        double ReadObj()
        {
            double res = ObjRTDA(maxRtda_);
            return res;
        }
        double ObjWeighted()
        {
            double overallRTDA = 0;
            for (uint i = 0; i < rtdaVec_.size(); i++)
                overallRTDA += ObjRTDA(rtdaVec_[i]);
            double sfOverall = sfVec_.sum();
            double res = ReadObj() + overallRTDA * weightInMpRTDA;
            if (considerSensorFusion != 0) // only used in RTSS21IC experiment
            {
                res += sfOverall * weightInMpSf + Barrier(sensorFusionTolerance - sfVec_.maxCoeff()) * weightInMpSfPunish;
                for (uint i = 0; i < rtdaVec_.size(); i++)
                    res += Barrier(FreshTol - ObjRTDA(maxRtda_[i])) * weightInMpRTDAPunish;
            }
            return res;
        }
    };

    template <class SchedulingAlgorithm>
    bool MakeProgress(IterationStatus<SchedulingAlgorithm> &statusPrev, IterationStatus<SchedulingAlgorithm> &statusCurr)
    {
        if (!statusCurr.schedulable_)
            return false;

        if (statusCurr.ObjWeighted() < statusPrev.ObjWeighted())
            return true;

        // if (overallObjCurr < overallObjPrev && ((double)rand() / (RAND_MAX)) < RandomAccept)
        //     return true;
        return false;
    }

    // exams whether switching j1 to the back of j2 is unschedulable
    bool WhetherSkipSwitch(const TaskSetInfoDerived &tasksInfo, const JobCEC &j1, const JobCEC &j2)
    {
        if (j1 == j2)
            return true;

        // Such switch always returns infeasible results
        if (j1.taskId == j2.taskId)
            return true;
        Interval v1(tasksInfo.tasks[j1.taskId].period * j1.jobId, tasksInfo.tasks[j1.taskId].period);
        Interval v2(tasksInfo.tasks[j2.taskId].period * j2.jobId, tasksInfo.tasks[j2.taskId].period);
        if (v2.start > v1.start + v1.length) //  - tasksInfo.tasks[j1.taskId].executionTime - tasksInfo.tasks[j2.taskId].executionTime
            return true;
        double c1 = tasksInfo.tasks[j1.taskId].executionTime;
        double c2 = tasksInfo.tasks[j2.taskId].executionTime;
        double d1 = tasksInfo.tasks[j1.taskId].deadline;

        double startTimeJ1Min = v2.start + c2;
        // schedulability requires satisfying this constraint: sMin + c1 <= s1+c1 <= D1
        if (startTimeJ1Min + c1 > v1.start + d1)
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

        LLint countMakeProgress = 0;
        LLint countIterationStatus = 0;
        auto ExamAndApplyUpdate = [&](JobOrderMultiCore jobOrderCurr)
        {
            IterationStatus<SchedulingAlgorithm> statusCurr(dagTasks, tasksInfo, jobOrderCurr, processorNum);
            countIterationStatus++;

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
                countMakeProgress++;
            }
        };

        auto start_time = std::chrono::system_clock::now();
        auto curr_time = std::chrono::system_clock::now();
        int64_t time_limit_in_seconds = makeProgressTimeLimit;
        if (time_limit_in_seconds < 0)
        {
            time_limit_in_seconds = INT64_MAX;
        }
        bool time_out_flag = false;

        auto CheckTimeOut = [&]()
        {
            curr_time = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= time_limit_in_seconds)
            {
                std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << time_limit_in_seconds << " seconds.\n\n";
                time_out_flag = true;
                return true;
            }
            return false;
        };

        LLint countOutermostWhileLoop = 0;
        while (findNewUpdate)
        {
            countOutermostWhileLoop++;
            if (time_out_flag)
                break;

            findNewUpdate = false;
            for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
            {
                if (time_out_flag)
                    break;
                for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                {
                    if (CheckTimeOut())
                        break;
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
                        if (time_out_flag)
                            break;
                        for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                        {
                            if (CheckTimeOut())
                                break;
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
                        if (CheckTimeOut())
                            break;
                        JobOrderMultiCore jobOrderCurr = statusPrev.jobOrder_;
                        jobOrderCurr.jobOrderSerial_[i] = !jobOrderCurr.jobOrderSerial_[i];
                        ExamAndApplyUpdate(jobOrderCurr); // update outside variables
                    }
                }
            }
        }

        ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.ReadObj(), statusPrev.processorJobVec_};
        scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
        // auto no_thing = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, statusPrev.jobOrder_, scheduleRes.processorJobVec_); // get the processor assignment

        if (doScheduleOptimization)
        {
            ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
            ScheduleResult result_after_optimization;
            schedule_optimizer.Optimize(dagTasks, scheduleRes);
            result_after_optimization = schedule_optimizer.getOptimizedResult();
            scheduleRes = result_after_optimization;
            if (!ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol))
            {
                CoutWarning("Found one unschedulable case after optimization!");
            }
        }

        scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
        std::cout << "Outermost while loop count: " << countOutermostWhileLoop << std::endl;
        std::cout << "Make progress count: " << countMakeProgress << std::endl;
        std::cout << "Candidate Iteration Status count: " << countIterationStatus << std::endl;
        return scheduleRes;
    }

} // namespace OrderOptDAG_SPACE
