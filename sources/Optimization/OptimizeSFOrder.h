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
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{
    struct IterationStatus
    {
        DAG_Model dagTasks_;
        SFOrder jobOrder_;
        int processorNum_;
        VectorDynamic startTimeVector_;
        std::vector<uint> processorJobVec_;
        std::vector<std::vector<RTDA>> rtdaVec_; // for each chain
        std::vector<RTDA> maxRtda_;              // for each chain
        // double objVal_;
        bool schedulable_; // only basic schedulability
        VectorDynamic sfVec_;

        IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, int processorNum) : dagTasks_(dagTasks), jobOrder_(jobOrder), processorNum_(processorNum)
        {
            // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
            processorJobVec_.clear();
            startTimeVector_ = SFOrderScheduling(dagTasks, tasksInfo, processorNum_, jobOrder_, processorJobVec_);

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
            schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_);
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

    bool MakeProgress(IterationStatus &statusPrev, IterationStatus &statusCurr)
    {
        if (!statusCurr.schedulable_)
            return false;
        if (statusCurr.ObjWeighted() < statusPrev.ObjWeighted())
            return true;
        return false;
    }

    bool WhetherSkipInsertStart(JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
    {
        if (startP > 0)
        {
            TimeInstance instancePrev = jobOrderCurr.instanceOrder_[startP - 1];
            // jP.ActivationTime <= jR.start <= jR.deadline - jR.executionTime
            if (GetActivationTime(instancePrev.job, tasksInfo) > GetDeadline(jobRelocate, tasksInfo) - tasksInfo.tasks[jobRelocate.taskId].executionTime)
                return true;
        }
        if (startP < tasksInfo.length * 2 - 1)
        {
            TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[startP + 1];
            //  jR.ActivationTime <= jR.start <= nextJ.Deadline
            double nextInstanceLeastFinishTime = GetDeadline(instanceAfter.job, tasksInfo);
            if (instanceAfter.type == 's')
                nextInstanceLeastFinishTime -= tasksInfo.tasks[instanceAfter.job.taskId].executionTime;
            if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                return true;
        }
        return false;
    }
    bool WhetherSkipInsertFinish(JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
    {
        if (finishP > 0)
        {
            TimeInstance instancePrev = jobOrderCurr.instanceOrder_[finishP - 1];
            // jP.ActivationTime <= jR.finish <= jR.deadline
            if (GetActivationTime(instancePrev.job, tasksInfo) > GetDeadline(jobRelocate, tasksInfo))
                return true;
        }
        if (finishP < tasksInfo.length * 2 - 1)
        {
            TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[finishP + 1];
            //  jR.ActivationTime <= jR.finish <= nextJ.Deadline
            double nextInstanceLeastFinishTime = GetDeadline(instanceAfter.job, tasksInfo);
            if (instanceAfter.type == 's')
                nextInstanceLeastFinishTime -= tasksInfo.tasks[instanceAfter.job.taskId].executionTime;
            if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                return true;
        }
        return false;
    }

    ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, int processorNum = coreNumberAva,
                                    boost::optional<ScheduleResult &> resOrderOptWithoutScheduleOpt = boost::none)
    {
        // srand(RandomDrawWeightMaxLoop);
        if (dagTasks.chains_.size() == 0)
            CoutWarning("No chain is provided for the given dag!");

        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        // VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
        if (debugMode == 1)
        {
            std::cout << "Initial schedule: " << std::endl;
            PrintSchedule(tasksInfo, initialSTV);
        }

        SFOrder jobOrderRef(tasksInfo, initialSTV);
        IterationStatus statusPrev(dagTasks, tasksInfo, jobOrderRef, processorNum);
        if (!statusPrev.schedulable_)
        {
            CoutWarning("Initial schedule is not schedulable!!!");
        }

        bool findNewUpdate = true;

        auto ExamAndApplyUpdate = [&](SFOrder jobOrderCurr)
        {
            IterationStatus statusCurr(dagTasks, tasksInfo, jobOrderCurr, processorNum);

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

        while (findNewUpdate)
        {
            if (time_out_flag)
                break;

            findNewUpdate = false;
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (LLint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                {
                    JobCEC jobRelocate(i, j);
                    SFOrder jobOrderCurr = statusPrev.jobOrder_;
                    jobOrderCurr.RemoveJob(jobRelocate);
                    // TODO: limit the range of possible choices
                    for (LLint startP = 0; startP < static_cast<LLint>(jobOrderCurr.size()); startP++)
                    {
                        if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurr))
                            break;
                        jobOrderCurr.InsertStart(jobRelocate, startP);
                        for (LLint finishP = startP; finishP < static_cast<LLint>(jobOrderCurr.size()); finishP++)
                        {
                            if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, jobOrderCurr))
                                break;
                            jobOrderCurr.InsertFinish(jobRelocate, finishP);
                            ExamAndApplyUpdate(jobOrderCurr);
                        }
                    }
                }
            }
        }
        ScheduleResult scheduleRes;
        scheduleRes.startTimeVector_ = statusPrev.startTimeVector_;
        // ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.ReadObj(), statusPrev.processorJobVec_};
        // auto no_thing = SFOrderScheduling(dagTasks, tasksInfo, processorNum, statusPrev.jobOrder_, scheduleRes.processorJobVec_); // get the processor assignment
        // if (resOrderOptWithoutScheduleOpt)
        // {
        //     scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
        //     *resOrderOptWithoutScheduleOpt = scheduleRes;
        // }

        // if (doScheduleOptimization)
        // {
        //     ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
        //     ScheduleResult result_after_optimization;
        //     schedule_optimizer.Optimize(dagTasks, scheduleRes);
        //     result_after_optimization = schedule_optimizer.getOptimizedResult();
        //     scheduleRes = result_after_optimization;
        //     if (!ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum))
        //     {
        //         CoutWarning("Found one unschedulable case after optimization!");
        //     }
        // }

        // scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
        return scheduleRes;
    }

} // namespace OrderOptDAG_SPACE
