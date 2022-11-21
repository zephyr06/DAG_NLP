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
        class DAGScheduleOptimizer
        {
        public:
            enum SFOrderStatus
            {
                Infeasible,
                InferiorFeasible,
                BetterFeasible
            };
            DAGScheduleOptimizer() {}

            DAGScheduleOptimizer(const DAG_Model &dagInput, const ScheduleOptions &scheduleOptions, double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit) : start_time(std::chrono::system_clock::now()), timeLimits(GlobalVariablesDAGOpt::makeProgressTimeLimit), dagTasks(dagInput), tasksInfo(TaskSetInfoDerived(dagTasks.tasks)), scheduleOptions(scheduleOptions)
            {
                if (dagTasks.chains_.size() == 0)
                    CoutWarning("No chain is provided for the given dag!");

                VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
                jobOrderRef = SFOrder(tasksInfo, initialSTV);
                statusPrev = IterationStatus<OrderScheduler, ObjectiveFunctionBase>(dagTasks, tasksInfo, jobOrderRef, scheduleOptions);
                if (!statusPrev.schedulable_)
                    CoutWarning("Initial schedule is not schedulable!!!");
            }

            ScheduleResult Optimize()
            {
                BeginTimer("OptimizeDAG");
                while (ifContinue())
                {
                    countOutermostWhileLoop++;
                    if (GlobalVariablesDAGOpt::debugMode == 1)
                        std::cout << "Outer loop count: " << countOutermostWhileLoop << std::endl;
                    findBetterJobOrderWithinIterations = false; // iterations stop unless a better job order is found

                    // search the tasks related to task chain at first
                    std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
                    for (int i : taskIdSet)
                    {
                        for (LLint j = 0; j < 0 + tasksInfo.sizeOfVariables[i] && (ifContinue()); j++)
                        {
                            JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);
                            ImproveJobOrderPerJob(jobRelocate);
                        }
                    }

                    if (!findBetterJobOrderWithinIterations)
                        break;
                }

                // TODO: optimize the following?
                std::vector<uint> processorJobVec;
                auto stv = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderRef, processorJobVec);
                ScheduleResult scheduleRes{jobOrderRef, statusPrev.startTimeVector_, statusPrev.schedulable_, ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, statusPrev.startTimeVector_, scheduleOptions), processorJobVec};
                EndTimer("OptimizeDAG");
                return scheduleRes;
            }

            inline SFOrder GetJobOrder() const
            {
                return jobOrderRef;
            }

            // data members
            std::chrono::high_resolution_clock::time_point start_time;
            double timeLimits;

            DAG_Model dagTasks;
            TaskSetInfoDerived tasksInfo;
            LLint countMakeProgress = 0;
            LLint countIterationStatus = 0;
            LLint countOutermostWhileLoop = 0;

            bool findBetterJobOrderWithinIterations = false;
            ScheduleOptions scheduleOptions;

            SFOrder jobOrderRef;
            IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusPrev;

        public:
            bool ifTimeout() const
            {
                auto curr_time = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= timeLimits)
                {
                    std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << timeLimits << " seconds.\n\n";
                    return true;
                }
                return false;
            }

            inline bool ifOptimal() const
            {
                return statusPrev.objWeighted_ == 0;
            }

            inline bool ifContinue() const { return (!ifTimeout()) && (!ifOptimal()); }

            bool ImproveJobOrderPerJob(const JobCEC &jobRelocate)
            {
                BeginTimer("ImproveJobOrderPerJob");
                JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, jobOrderRef, tasksInfo);
                for (LLint startP = jobStartFinishInstActiveRange.minIndex; startP < jobStartFinishInstActiveRange.maxIndex && ifContinue(); startP++)
                {
                    BeginTimer("SFOrderCopy");
                    SFOrder jobOrderCurrForStart = jobOrderRef;
                    EndTimer("SFOrderCopy");
                    jobOrderCurrForStart.RemoveJob(jobRelocate);
                    if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurrForStart))
                        continue;

                    jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
                    double accumLengthMin = 0;
                    for (LLint finishP = startP + 1; finishP < jobStartFinishInstActiveRange.maxIndex + 1 && ifContinue(); finishP++)
                    {
                        if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, jobOrderRef))
                            continue;
                        if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP, tasksInfo, jobOrderCurrForStart, startP))
                            break;

                        SFOrder &jobOrderCurrForFinish = jobOrderCurrForStart;
                        jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                        SFOrderStatus sfOrderStatus = UpdateStatus(jobOrderCurrForFinish, jobStartFinishInstActiveRange, finishP);

                        if (sfOrderStatus == SFOrderStatus::BetterFeasible)
                        {
                            findBetterJobOrderWithinIterations = true;
                            // statusPrev = statusCurr;
                            // jobOrderRef = jobOrderCurrForFinish;
                            if (GlobalVariablesDAGOpt::debugMode == 1)
                            {
                                std::cout << "Make progress!" << std::endl;
                            }
                            countMakeProgress++;
                        }
                        else if (sfOrderStatus == SFOrderStatus::InferiorFeasible)
                            jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
                        else
                        {
                            jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
                            break;
                        }
                    }
                }

                EndTimer("ImproveJobOrderPerJob");
                return findBetterJobOrderWithinIterations;
            }

            // Compare against statusPrev built from jobOrderRef, and update statusPrev and jobOrderRef if success and return true
            // TODO: should jobOrderCurrForFinish be const?
            SFOrderStatus UpdateStatus(SFOrder &jobOrderCurrForFinish, JobGroupRange &jobStartFinishInstActiveRange, LLint finishP)
            {
                // check whether the small job order under influence is unschedulable
                if (SubGroupSchedulabilityCheck(jobStartFinishInstActiveRange, jobOrderRef, jobOrderCurrForFinish, finishP, dagTasks, tasksInfo, scheduleOptions.processorNum_))
                    return SFOrderStatus::Infeasible;
                else
                {
                    if (SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderCurrForFinish)(0) == -1)
                        return SFOrderStatus::Infeasible;
                }

                IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions);
                countIterationStatus++;

                if (MakeProgress<OrderScheduler>(statusPrev, statusCurr))
                {
                    statusPrev = statusCurr;
                    jobOrderRef = jobOrderCurrForFinish;
                    return SFOrderStatus::BetterFeasible;
                }
                else
                    return SFOrderStatus::InferiorFeasible;
            }

        }; // class DAGScheduleOptimizer

        template <typename OrderScheduler, typename ObjectiveFunctionBase>
        ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions, double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit)
        {
            DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase> dagScheduleOptimizer(dagTasks, scheduleOptions, timeLimits);
            const TaskSetInfoDerived &tasksInfo = dagScheduleOptimizer.tasksInfo;
            const SFOrder &jobOrderRef = dagScheduleOptimizer.GetJobOrder();
            ScheduleResult scheduleRes = dagScheduleOptimizer.Optimize();

            // std::vector<uint> processorJobVec;
            // auto stv = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrderRef, processorJobVec);

            if (scheduleOptions.doScheduleOptimization_)
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

            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, scheduleOptions.processorNum_, GlobalVariablesDAGOpt::sensorFusionTolerance, GlobalVariablesDAGOpt::freshTol);
            std::cout << "Outermost while loop count: " << dagScheduleOptimizer.countOutermostWhileLoop << std::endl;
            std::cout << "Make progress count: " << dagScheduleOptimizer.countMakeProgress << std::endl;
            std::cout << Color::blue << "Candidate Iteration Status count: " << dagScheduleOptimizer.countIterationStatus << Color::def << std::endl;
            std::cout << "infeasibleCount: " << infeasibleCount << std::endl;
            std::cout << "Total number of variables: " << tasksInfo.length << std::endl;
            scheduleRes.countOutermostWhileLoop_ = dagScheduleOptimizer.countOutermostWhileLoop;
            scheduleRes.countMakeProgress_ = dagScheduleOptimizer.countMakeProgress;
            scheduleRes.countIterationStatus_ = dagScheduleOptimizer.countIterationStatus;

            return scheduleRes;
        }
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
