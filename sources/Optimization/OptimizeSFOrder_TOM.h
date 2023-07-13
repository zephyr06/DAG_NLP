#pragma once
#include <algorithm>  // for copy() and assign()
#include <iterator>   // for back_inserter

#include "sources/Optimization/IndependentAnalysis.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/Interval.h"
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/ObjectiveFunctions.h"
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Factors/SensorFusion_Analyze.h"
#include "sources/Optimization/IndependentAnalysis.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ProcessorAssignment.h"
#include "sources/Optimization/SFOrder.h"
// #include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/SkipUnschedulablePermutations.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {

std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks);

void CheckValidDAGTaskSetGivenObjType(const DAG_Model &dagTasks,
                                      std::string obj_type);

// return the range of index that the start instnace of jobRelocate could be,
// inclusive on both ends; the index of JobGroupRange is based on the original
// jobOrderRef without removing jobRelocate To use the found JobGroupRange, user
// needs to check bounds on SFOrder;
JobGroupRange FindJobActivateRange(const JobCEC &jobRelocate,
                                   SFOrder &jobOrderRef,
                                   const TaskSetInfoDerived &tasksInfo);

template <typename OrderScheduler, typename ObjectiveFunctionBase>
class DAGScheduleOptimizer {
   public:
    DAGScheduleOptimizer() {}

    DAGScheduleOptimizer(
        const DAG_Model &dagInput, const ScheduleOptions &scheduleOptions,
        double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT)
        : start_time(std::chrono::system_clock::now()),
          timeLimits(GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT),
          dagTasks(dagInput),
          tasksInfo(TaskSetInfoDerived(dagTasks.tasks)),
          scheduleOptions(scheduleOptions) {
        if (dagTasks.chains_.size() == 0)
            CoutWarning("No chain is provided for the given dag!");

        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks, tasksInfo, scheduleOptions.processorNum_);

        InitializeStatus(initialSTV);

        if (!statusPrev.schedulable_)
            CoutWarning("Initial schedule is not schedulable!!!");

        if (GlobalVariablesDAGOpt::debugMode == 1) {
            std::cout << "Initial schedule: " << std::endl;
            PrintSchedule(tasksInfo, initialSTV);
            std::cout << initialSTV << std::endl;
            std::cout << "Initial SF order: " << std::endl;
            jobOrderRef.print();
        }

#ifdef PROFILE_CODE
        std::remove(GetStatsSeqFileName().c_str());
#endif
    }

    ScheduleResult Optimize() {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif
        do {
            countOutermostWhileLoop++;
            if (GlobalVariablesDAGOpt::debugMode == 1)
                std::cout << "Outer loop count: " << countOutermostWhileLoop
                          << std::endl;
            findBetterJobOrderWithinIterations =
                false;  // iterations stop unless a better job order is found

            // search the tasks related to task chain at first
            std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
            for (int i : taskIdSet) {
                for (LLint j = 0;
                     j < tasksInfo.sizeOfVariables[i] && (ifContinue()); j++) {
                    JobCEC jobRelocate(i, j);
                    ImproveJobOrderPerJob(jobRelocate);
                }
            }

            if (!findBetterJobOrderWithinIterations)
                break;
        } while (ifContinue() && findBetterJobOrderWithinIterations);

             auto scheduleRes = GetScheduleResult();
#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
        return scheduleRes;
    }

    ScheduleResult GetScheduleResult() {
        std::vector<uint> processorJobVec;
        auto stv = OrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderRef, processorJobVec,
            ObjectiveFunctionBase::type_trait);
        return ScheduleResult(
            jobOrderRef, statusPrev.startTimeVector_, statusPrev.schedulable_,
            ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo,
                                           statusPrev.startTimeVector_,
                                           scheduleOptions),
            processorJobVec);
    }
    void ImproveJobOrderPerJob(const JobCEC &jobRelocate) {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif

        JobGroupRange jobIndexRange =
            FindJobActivateRange(jobRelocate, jobOrderRef, tasksInfo);
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusBestFound =
            statusPrev;
        jobOrderRef.EstablishJobSFMap();
        SFOrder jobOrderBestFound = jobOrderRef;

        for (LLint startP = jobIndexRange.minIndex;
             startP <= jobIndexRange.maxIndex - 2 &&
             startP <= int(tasksInfo.length) * 2 - 2 && ifContinue();
             startP++) {
            SFOrder jobOrderCurrForStart = jobOrderRef;
            jobOrderCurrForStart.RemoveJob(jobRelocate);
            if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo,
                                       jobOrderCurrForStart))
                continue;

            jobOrderCurrForStart.InsertStart(
                jobRelocate, startP);  // must insert start first
            double accumLengthMin = 0;
            for (LLint finishP = startP + 1;
                 finishP <=
                     std::min(jobIndexRange.maxIndex - 1,
                              static_cast<int>(tasksInfo.length) * 2 - 1) &&
                 ifContinue();
                 finishP++) {
                if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo,
                                            jobOrderRef))
                    continue;

                SFOrder jobOrderCurrForFinish =
                    jobOrderCurrForStart;  //  copying by value is sometimes
                                           //  faster
                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                if (WhetherFinishPositionInfeasible(
                        accumLengthMin, jobRelocate, startP, finishP,
                        jobOrderCurrForStart, jobOrderCurrForFinish))
                    break;

                CompareAndUpdateStatus(jobOrderCurrForFinish, statusBestFound,
                                       jobOrderBestFound);

                // TODO: Avoid update job order’s internal index
                jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
            }
            // jobOrderCurrForStart.RemoveStart(jobRelocate, startP);
        }

        if (statusPrev.objWeighted_ != statusBestFound.objWeighted_) {
            ApplyBestAdjacentJobOrderOfOneJob(statusBestFound,
                                              jobOrderBestFound);
        }

#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
        return;
    }
    void ApplyBestAdjacentJobOrderOfOneJob(
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) {
        statusPrev = statusBestFound;
        auto jobOrderRefNew = SFOrder(
            tasksInfo,
            statusBestFound.startTimeVector_);  // avoid incorrect job order
        if (jobOrderRefNew != jobOrderBestFound)
            CheckLPConsistency(jobOrderRefNew, statusBestFound,
                               jobOrderBestFound);
        jobOrderRef = jobOrderBestFound;
        findBetterJobOrderWithinIterations = true;
        countMakeProgress++;
    }

    void CheckLPConsistency(
        SFOrder &jobOrderRefNew,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) {
        std::vector<uint> processorJobVec1;
        auto stvNew = LPOrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderRefNew,
            processorJobVec1, ObjectiveFunctionBase::type_trait);
        if (stvNew != statusBestFound.startTimeVector_ &&
            (stvNew - statusBestFound.startTimeVector_).norm() >
                1e0)  // print some info
        {
            std::cout << "Start time vector of jobOrderBestFound:\n"
                      << statusBestFound.startTimeVector_ << "\n\n\n";
            jobOrderBestFound.print();
            PrintSchedule(tasksInfo, statusBestFound.startTimeVector_);
            std::cout << "\n\n\n";
            jobOrderRefNew.print();
            PrintSchedule(tasksInfo, stvNew);
            CoutError("Inconsistent job order!");
        }
    }

    bool WhetherFinishPositionInfeasible(double &accumLengthMin,
                                         JobCEC jobRelocate, LLint startP,
                                         LLint finishP,
                                         SFOrder &jobOrderCurrForStart,
                                         SFOrder &jobOrderCurrForFinish) const {
        if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP,
                                      tasksInfo, jobOrderCurrForStart, startP))
            return true;

        std::vector<uint> processorJobVec;
        if (!ProcessorAssignment::AssignProcessor(
                tasksInfo, jobOrderCurrForFinish, scheduleOptions.processorNum_,
                processorJobVec)) {
            jobOrderCurrForFinish.RemoveInstance(jobRelocate, finishP);
            return true;
        }
        return false;
    }

    // Compare against statusPrev built from jobOrderRef, and update statusPrev
    // and jobOrderRef if success and return true
    // TODO: SubGroupSchedulabilityCheck is temporarily removed, which is used
    // to check whether the small job order under influence is unschedulable
    bool CompareAndUpdateStatus(
        SFOrder &jobOrderCurrForFinish,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) {
        VectorDynamic startTimeVector;
        std::vector<uint> processorJobVec;
        startTimeVector = OrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderCurrForFinish,
            processorJobVec, ObjectiveFunctionBase::type_trait);

        bool schedulable = ExamBasic_Feasibility(
            dagTasks, tasksInfo, startTimeVector, processorJobVec,
            scheduleOptions.processorNum_);
        if (!schedulable) {
            infeasibleCount++;
            return false;
        }

        return WhetherMakeProgressAndUpdateBestFoundStatus(
            startTimeVector, processorJobVec, schedulable,
            jobOrderCurrForFinish, statusBestFound, jobOrderBestFound);
    }

    bool WhetherMakeProgressAndUpdateBestFoundStatus(
        const VectorDynamic &startTimeVector,
        const std::vector<uint> &processorJobVec, bool schedulable,
        SFOrder &jobOrderCurrForFinish,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) {
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(
            dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions,
            startTimeVector, processorJobVec, schedulable);
        countIterationStatus++;
        if (MakeProgress<OrderScheduler>(statusBestFound, statusCurr)) {
            statusBestFound = statusCurr;
            jobOrderBestFound = jobOrderCurrForFinish;
            if (GlobalVariablesDAGOpt::debugMode == 1) {
                std::cout << "Make progress!" << std::endl;
                jobOrderCurrForFinish.print();
                std::cout << "start time vector: \n"
                          << statusBestFound.startTimeVector_ << "\n";
                PrintSchedule(tasksInfo, statusBestFound.startTimeVector_);
                // double objCurr =
                //     ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo,
                //     startTimeVector, scheduleOptions);
                // double objPrev =
                //     ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo,
                //     statusPrev.startTimeVector_, scheduleOptions);
                // int a = 1;
            }
            return true;
        } else {  // not make progress
            return false;
        }
    }

    void InitializeStatus(const VectorDynamic &startTimeVector) {
        jobOrderRef = SFOrder(tasksInfo, startTimeVector);
        statusPrev = IterationStatus<OrderScheduler, ObjectiveFunctionBase>(
            dagTasks, tasksInfo, jobOrderRef, scheduleOptions);
    }

    inline SFOrder GetJobOrder() const { return jobOrderRef; }

    bool ifTimeout() const {
        auto curr_time = std::chrono::system_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(curr_time -
                                                             start_time)
                .count() >= timeLimits) {
            std::cout
                << "\nTime out when running OptimizeOrder. Maximum time is "
                << timeLimits << " seconds.\n\n";
            return true;
        }
        return false;
    }

    inline bool ifOptimal() const { return statusPrev.objWeighted_ == 0; }

    inline bool ifContinue() const { return (!ifTimeout()) && (!ifOptimal()); }

    // data members
    std::chrono::high_resolution_clock::time_point start_time;
    double timeLimits;

    DAG_Model dagTasks;
    TaskSetInfoDerived tasksInfo;
    LLint countMakeProgress = 0;
    LLint countIterationStatus = 0;
    LLint countOutermostWhileLoop = 0;
    int infeasibleCount = 0;

    bool findBetterJobOrderWithinIterations = false;
    ScheduleOptions scheduleOptions;

    SFOrder jobOrderRef;
    IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusPrev;
};  // class DAGScheduleOptimizer

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel(
    DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
    CheckValidDAGTaskSetGivenObjType(dagTasks,
                                     ObjectiveFunctionBase::type_trait);
    DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>
        dagScheduleOptimizer(dagTasks, scheduleOptions, timeLimits);
    const TaskSetInfoDerived &tasksInfo = dagScheduleOptimizer.tasksInfo;
    const SFOrder &jobOrderRef = dagScheduleOptimizer.GetJobOrder();
    ScheduleResult scheduleRes = dagScheduleOptimizer.Optimize();

    if (ObjectiveFunctionBase::type_trait == "ReactionTimeObj" ||
        ObjectiveFunctionBase::type_trait == "DataAgeObj" ||
        ObjectiveFunctionBase::type_trait == "SensorFusionObj")
        scheduleRes.schedulable_ = ExamBasic_Feasibility(
            dagTasks, tasksInfo, scheduleRes.startTimeVector_,
            scheduleRes.processorJobVec_, scheduleOptions.processorNum_);
    else
        CoutError("Unknown obj type in ScheduleDAGModel");
    std::cout << "Outermost while loop count: "
              << dagScheduleOptimizer.countOutermostWhileLoop << std::endl;
    std::cout << "Make progress count: "
              << dagScheduleOptimizer.countMakeProgress << std::endl;
    std::cout << Color::blue << "Candidate Iteration Status count: "
              << dagScheduleOptimizer.countIterationStatus << Color::def
              << std::endl;
    std::cout << "infeasibleCount: " << dagScheduleOptimizer.infeasibleCount
              << std::endl;
    std::cout << "Total number of variables: " << tasksInfo.length << std::endl;
    scheduleRes.countOutermostWhileLoop_ =
        dagScheduleOptimizer.countOutermostWhileLoop;
    scheduleRes.countMakeProgress_ = dagScheduleOptimizer.countMakeProgress;
    scheduleRes.countIterationStatus_ =
        dagScheduleOptimizer.countIterationStatus;

    return scheduleRes;
}
}  // namespace OptimizeSF
}  // namespace OrderOptDAG_SPACE
