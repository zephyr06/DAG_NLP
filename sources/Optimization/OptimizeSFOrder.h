#pragma once
#include <algorithm> // for copy() and assign()
#include <iterator>  // for back_inserter

#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/SkipUnschedulablePermutations.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ObjectiveFunctions.h"
// #include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE
{
    // template <typename OrderScheduler>
    namespace OptimizeSF
    {

        std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks);
        JobGroupRange FindJobActivateRange(const JobCEC &jobRelocate, SFOrder &jobOrderRef, const TaskSetInfoDerived &tasksInfo);

        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        inline bool FoundOptimal(const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusPrev)
        {
            return statusPrev.objWeighted_ == 0;
        }

        bool CheckTimeOut(const std::chrono::high_resolution_clock::time_point &start_time, const double &time_limit_in_seconds)
        {
            auto curr_time = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= time_limit_in_seconds)
            {
                std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << time_limit_in_seconds << " seconds.\n\n";
                return true;
            }
            return false;
        }

        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        bool CheckIterationContinue(const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusPrev, const std::chrono::high_resolution_clock::time_point &start_time, const double &time_limit_in_seconds)
        {
            if (CheckTimeOut(start_time, time_limit_in_seconds) || FoundOptimal(statusPrev))
                return false;
            else
                return true;
        }
        // jobOrderRef and statusPrev will be updated in-place
        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        bool FindSFOrderPerJob(const JobCEC &jobRelocate, SFOrder &jobOrderRef, IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusPrev, DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const ScheduleOptions &scheduleOptions, LLint &countMakeProgress, LLint &countIterationStatus, const std::chrono::high_resolution_clock::time_point &start_time, const double &time_limit_in_seconds)
        {
            bool findBetterJobOrderWithinIterations = false;
            JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, jobOrderRef, tasksInfo);
            for (LLint startP = jobStartFinishInstActiveRange.minIndex; startP < jobStartFinishInstActiveRange.maxIndex && CheckIterationContinue<OrderScheduler, ObjectiveFunctionBase>(statusPrev, start_time, time_limit_in_seconds); startP++)
            {
                // TODO: this part can be optimized, though not very necessary
                BeginTimer("SFOrderCopy");
                SFOrder jobOrderCurrForStart = jobOrderRef;
                EndTimer("SFOrderCopy");
                jobOrderCurrForStart.RemoveJob(jobRelocate);
                if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurrForStart))
                    continue;

                jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
                double accumLengthMin = 0;
                for (LLint finishP = startP + 1; finishP < jobStartFinishInstActiveRange.maxIndex + 1 && CheckIterationContinue<OrderScheduler, ObjectiveFunctionBase>(statusPrev, start_time, time_limit_in_seconds); finishP++)
                {
                    if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, jobOrderRef))
                        continue;
                    if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP, tasksInfo, jobOrderCurrForStart, startP))
                        break;

                    SFOrder &jobOrderCurrForFinish = jobOrderCurrForStart;
                    jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                    // check whether the small job order under influence is unschedulable
                    if (SubGroupSchedulabilityCheck(jobStartFinishInstActiveRange, jobOrderRef, jobOrderCurrForFinish, finishP, dagTasks, tasksInfo, scheduleOptions.processorNum_))
                    {
                        break;
                    }
                    else
                    {
                        BeginTimer("PrevSchedulabilityCheck");
                        if (SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderCurrForFinish)(0) == -1)
                        {
                            EndTimer("PrevSchedulabilityCheck");
                            break;
                        }
                        EndTimer("PrevSchedulabilityCheck");
                    }

                    IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions);
                    countIterationStatus++;

                    if (MakeProgress<OrderScheduler>(statusPrev, statusCurr))
                    {
                        findBetterJobOrderWithinIterations = true;
                        statusPrev = statusCurr;
                        jobOrderRef = jobOrderCurrForFinish;
                        if (debugMode == 1)
                        {
                            std::cout << "Make progress!" << std::endl;
                            // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                        }
                        countMakeProgress++;
                    }
                    else
                    {
                        // restore jobOrderCurrForStart for next iteration
                        jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
                    }
                }
            }
            return findBetterJobOrderWithinIterations;
        }

        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions, double timeLimits = makeProgressTimeLimit)
        {
            std::vector<int> countSubJobOrderLength;
            if (dagTasks.chains_.size() == 0)
                CoutWarning("No chain is provided for the given dag!");

            TaskSetInfoDerived tasksInfo(dagTasks.tasks);
            VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
            SFOrder jobOrderRef(tasksInfo, initialSTV);
            if (debugMode == 1)
            {
                std::cout << "Initial schedule: " << std::endl;
                PrintSchedule(tasksInfo, initialSTV);
                std::cout << initialSTV << std::endl;
                std::cout << "Initial SF order: " << std::endl;
                jobOrderRef.print();
            }
            IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusPrev(dagTasks, tasksInfo, jobOrderRef, scheduleOptions);
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Initial schedule is not schedulable!!!");
            }

            auto start_time = std::chrono::system_clock::now();
            int64_t time_limit_in_seconds = timeLimits;
            if (time_limit_in_seconds < 0)
            {
                time_limit_in_seconds = INT64_MAX;
            }

            // auto CheckTimeOut = [&start_time, &time_limit_in_seconds]()
            // {
            //     auto curr_time = std::chrono::system_clock::now();
            //     if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= time_limit_in_seconds)
            //     {
            //         std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << time_limit_in_seconds << " seconds.\n\n";
            //         return true;
            //     }
            //     return false;
            // };

            // auto CheckIterationContinue = [&statusPrev]()
            // {
            //     if (CheckTimeOut(start_time, time_limit_in_seconds) || FoundOptimal(statusPrev))
            //         return false;
            //     else
            //         return true;
            // };

            LLint countMakeProgress = 0;
            LLint countIterationStatus = 0;
            LLint countOutermostWhileLoop = 0;
            while (CheckIterationContinue<OrderScheduler, ObjectiveFunctionBase>(statusPrev, start_time, time_limit_in_seconds))
            {
                countOutermostWhileLoop++;
                bool findBetterJobOrderWithinIterations = false; // iterations stop unless a better job order is found

                // search the tasks related to task chain at first
                std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
                BeginTimer("inner_for_job");
                for (int i : taskIdSet)
                    for (LLint j = 0; j < 0 + tasksInfo.sizeOfVariables[i] && CheckIterationContinue<OrderScheduler, ObjectiveFunctionBase>(statusPrev, start_time, time_limit_in_seconds); j++)
                    {
                        JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);

                        findBetterJobOrderWithinIterations = findBetterJobOrderWithinIterations || FindSFOrderPerJob(jobRelocate, jobOrderRef, statusPrev, dagTasks, tasksInfo, scheduleOptions, countMakeProgress, countIterationStatus, start_time, time_limit_in_seconds);
                    }
                if (!findBetterJobOrderWithinIterations)
                    break;
                EndTimer("inner_for_job");
            }
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Optimize SFOrder return with unschedulable result!");
            }

            std::vector<uint> processorJobVec;
            auto stv = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderRef, processorJobVec);

            // TODO: this constructor could be an empty one
            ScheduleResult scheduleRes{jobOrderRef, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.objWeighted_, processorJobVec};

            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, processorJobVec, scheduleOptions.processorNum_, sensorFusionTolerance, freshTol);
            scheduleRes.obj_ = ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, statusPrev.startTimeVector_, scheduleOptions);

            if (scheduleOptions.doScheduleOptimization_ && !FoundOptimal(statusPrev))
            {
                if (!scheduleOptions.considerSensorFusion_ || !scheduleRes.schedulable_)
                {
                    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
                    ScheduleResult result_after_optimization;
                    if (scheduleOptions.considerSensorFusion_)
                    {
                        schedule_optimizer.OptimizeObjWeighted(dagTasks, scheduleRes);
                        result_after_optimization = schedule_optimizer.getOptimizedResult();
                        if (result_after_optimization.objWeighted_ < scheduleRes.objWeighted_)
                        {
                            scheduleRes = result_after_optimization;
                            std::vector<RTDA> rtda_vector;
                            for (auto chain : dagTasks.chains_)
                            {
                                auto res = GetRTDAFromSingleJob(tasksInfo, chain, scheduleRes.startTimeVector_);
                                RTDA resM = GetMaxRTDA(res);
                                rtda_vector.push_back(resM);
                            }
                            scheduleRes.obj_ = ObjRTDA(rtda_vector);
                        }
                    }
                    else
                    {
                        schedule_optimizer.Optimize(dagTasks, scheduleRes);
                        scheduleRes = schedule_optimizer.getOptimizedResult();
                    }
                }
            }

            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, scheduleOptions.processorNum_, sensorFusionTolerance, freshTol);
            std::cout << "Outermost while loop count: " << countOutermostWhileLoop << std::endl;
            std::cout << "Make progress count: " << countMakeProgress << std::endl;
            std::cout << Color::blue << "Candidate Iteration Status count: " << countIterationStatus << Color::def << std::endl;
            std::cout << "infeasibleCount: " << infeasibleCount << std::endl;
            // std::cout << "Average sub-job group length: " << Average(countSubJobOrderLength) << std::endl;
            // std::cout << "Maximum sub-job group length: " << *std::max_element(countSubJobOrderLength.begin(), countSubJobOrderLength.end()) << std::endl;
            std::cout << "Total number of variables: " << tasksInfo.length << std::endl;
            scheduleRes.countOutermostWhileLoop_ = countOutermostWhileLoop;
            scheduleRes.countMakeProgress_ = countMakeProgress;
            scheduleRes.countIterationStatus_ = countIterationStatus;

            return scheduleRes;
        }
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
