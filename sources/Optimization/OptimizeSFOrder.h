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

        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions, boost::optional<ScheduleResult &> resOrderOptWithoutScheduleOpt = boost::none)
        {

            std::vector<int> countSubJobOrderLength;
            // srand(RandomDrawWeightMaxLoop);
            if (dagTasks.chains_.size() == 0)
                CoutWarning("No chain is provided for the given dag!");

            TaskSet &tasks = dagTasks.tasks;
            TaskSetInfoDerived tasksInfo(tasks);
            // VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum);
            VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
            if (debugMode == 1)
            {
                std::cout << "Initial schedule: " << std::endl;
                PrintSchedule(tasksInfo, initialSTV);
                std::cout << initialSTV << std::endl;
            }

            SFOrder jobOrderRef(tasksInfo, initialSTV);
            if (debugMode == 1)
            {
                std::cout << "Initial SF order: " << std::endl;
                jobOrderRef.print();
            }
            IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusPrev(dagTasks, tasksInfo, jobOrderRef, scheduleOptions);
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Initial schedule is not schedulable!!!");
            }
            bool foundOptimal = false;
            bool findNewUpdate = true;
            LLint countMakeProgress = 0;
            LLint countIterationStatus = 0;

            if (statusPrev.objWeighted_ == 0)
                foundOptimal = true;

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
            // int jobWithMaxChain = FindLongestChainJobIndex(statusPrev)[0];
            int jobWithMaxChain = 0;

            LLint countOutermostWhileLoop = 0;
            while (findNewUpdate)
            {
                countOutermostWhileLoop++;
                if (time_out_flag || foundOptimal)
                    break;

                findNewUpdate = false;

                // search the tasks related to task chain at first
                std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
                BeginTimer("inner_for_job");
                for (int i : taskIdSet)
                    for (LLint j = jobWithMaxChain; j < jobWithMaxChain + tasksInfo.sizeOfVariables[i]; j++)
                    {
                        if (time_out_flag || foundOptimal)
                            break;
                        JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);
                        LLint prevJobIndex = 0, nextJobIndex = static_cast<LLint>(statusPrev.jobOrder_.size() - 1);
                        if (jobRelocate.jobId > 0)
                        {
                            JobCEC prevJob(jobRelocate.taskId, jobRelocate.jobId - 1);
                            prevJobIndex = statusPrev.jobOrder_.GetJobFinishInstancePosition(prevJob);
                        }
                        if (jobRelocate.jobId < tasksInfo.sizeOfVariables[jobRelocate.taskId] - 1)
                        {
                            JobCEC nextJob(jobRelocate.taskId, jobRelocate.jobId + 1);
                            nextJobIndex = std::min(statusPrev.jobOrder_.GetJobStartInstancePosition(nextJob) + 1, nextJobIndex); // actually, I'm not sure why do we need this "+1", but it doesn't hurt to search for a few more
                        }

                        JobGroupRange jobGroup(prevJobIndex, prevJobIndex);
                        for (LLint startP = prevJobIndex; startP < nextJobIndex; startP++)
                        {
                            BeginTimer("inner_for_start");
                            if (time_out_flag || foundOptimal)
                                break;

                            jobGroup.minIndex = startP;

                            // TODO: this part can be optimized, though not very necessary
                            BeginTimer("SFOrderCopy");
                            SFOrder jobOrderCurrForStart = statusPrev.jobOrder_;
                            EndTimer("SFOrderCopy");
                            jobOrderCurrForStart.RemoveJob(jobRelocate);
                            if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurrForStart))
                                continue;

                            jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
                            double accumLengthMin = 0;
                            for (LLint finishP = startP + 1; finishP < nextJobIndex + 1; finishP++)
                            {
                                if (CheckTimeOut())
                                    break;
                                if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, statusPrev.jobOrder_))
                                    continue;
                                if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP, tasksInfo, jobOrderCurrForStart, startP))
                                    break;

                                SFOrder &jobOrderCurrForFinish = jobOrderCurrForStart;
                                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                                // check whether the small job order under influence is unschedulable
                                if (SubGroupSchedulabilityCheck(jobGroup, statusPrev, jobOrderCurrForFinish, finishP, dagTasks, tasksInfo, scheduleOptions.processorNum_))
                                {
                                    break;
                                }
                                else
                                {
                                    BeginTimer("PrevSchedulabilityCheck");
                                    if (SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderCurrForFinish)(0) == -1)
                                    {
                                        break;
                                    }
                                    EndTimer("PrevSchedulabilityCheck");
                                }

                                BeginTimer("IterationStatusCreate");
                                IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions);
                                EndTimer("IterationStatusCreate");
                                countIterationStatus++;

                                if (MakeProgress<OrderScheduler>(statusPrev, statusCurr))
                                {
                                    findNewUpdate = true;
                                    statusPrev = statusCurr;
                                    if (debugMode == 1)
                                    {
                                        std::cout << "Make progress!" << std::endl;
                                        // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                                    }
                                    countMakeProgress++;
                                    if (statusCurr.objWeighted_ == 0)
                                        foundOptimal = true;
                                }
                                else
                                {
                                    // restore jobOrderCurrForStart for next iteration
                                    jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
                                }
                                if (foundOptimal)
                                    break;
                            }
                            EndTimer("inner_for_start");
                        }
                    }
                // std::cout << "Finish one big while loop!" << std::endl;
                EndTimer("inner_for_job");
            }
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Optimize SFOrder return with unschedulable result!");
            }

            std::vector<uint> processorJobVec;
            auto stv = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions.processorNum_, statusPrev.jobOrder_, processorJobVec);

            ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.objWeighted_, processorJobVec};
            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, processorJobVec, scheduleOptions.processorNum_, sensorFusionTolerance, freshTol);
            scheduleRes.objWeighted_ = statusPrev.objWeighted_;

            if (doScheduleOptimization && !foundOptimal)
            {
                if (!considerSensorFusion || !scheduleRes.schedulable_)
                {
                    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
                    ScheduleResult result_after_optimization;
                    if (considerSensorFusion)
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
