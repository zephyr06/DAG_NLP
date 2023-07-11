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
#define CHECK_IA_CORRECTNESS

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
        double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit)
        : start_time(std::chrono::system_clock::now()),
          timeLimits(GlobalVariablesDAGOpt::makeProgressTimeLimit),
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

    // bool WhetherJobInfluenceChainLength(JobCEC jobRelocate) {
    //     for (uint kk = 0; kk < longestJobChains_.size(); kk++) {
    //         JobCEC sourceJob = longestJobChains_[kk][0];
    //         JobCEC sinkJob =
    //             longestJobChains_[kk][longestJobChains_[kk].size() - 1];
    //         bool influenceSource = WhetherJobStartLater(
    //             sourceJob, jobRelocate, jobGroupMap_, jobOrderRef, tasksInfo,
    //             statusPrev.startTimeVector_);
    //         bool influenceSink = WhetherJobStartEarlier(
    //             sinkJob, jobRelocate, jobGroupMap_, jobOrderRef, tasksInfo,
    //             statusPrev.startTimeVector_);
    //         if (influenceSource || influenceSink) {
    //             // jobOrderRef.print();
    //             return true;
    //         }
    //     }
    //     return false;
    // }

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

        std::vector<uint> processorJobVec;
        auto stv = OrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderRef, processorJobVec,
            ObjectiveFunctionBase::type_trait);
        ScheduleResult scheduleRes{
            jobOrderRef, statusPrev.startTimeVector_, statusPrev.schedulable_,
            ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo,
                                           statusPrev.startTimeVector_,
                                           scheduleOptions),
            processorJobVec};
#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
        return scheduleRes;
    }

    bool ImproveJobOrderPerJob(const JobCEC &jobRelocate) {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif
        // TODO: skip this function if jobRelocate is not an active job
        // #ifndef CHECK_IA_CORRECTNESS  // perform some Independent Analysis to
        // speedup
        //                               // iterations
        //         if (!independent_analysis_.IfActiveJob(jobRelocate))
        //             return false;
        // #endif
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

                // Independence analysis
                if (GlobalVariablesDAGOpt::enableIndependentAnalysis) {
                    // TODO: add WhetherJobInfluenceChainLength into IA?
                    if_IA_skip = independent_analysis_.WhetherSafeSkip(
                        jobRelocate, startP, finishP, jobOrderRef);
#ifndef CHECK_IA_CORRECTNESS
                    if (if_IA_skip)
                        continue;
#endif
                }

                SFOrder jobOrderCurrForFinish =
                    jobOrderCurrForStart;  //  copying by value is sometimes
                                           //  faster
                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                // TODO: rename this function?
                if (BreakFinishPermutation(accumLengthMin, jobRelocate, startP,
                                           finishP, jobOrderCurrForStart,
                                           jobOrderCurrForFinish))
                    break;

                // bool findImprove =
                CompareAndUpdateStatus(jobOrderCurrForFinish, statusBestFound,
                                       jobOrderBestFound);

                // TODO: Avoid update job order’s internal index
                jobOrderCurrForFinish.RemoveFinish(jobRelocate, finishP);
            }
            // jobOrderCurrForStart.RemoveStart(jobRelocate, startP);
        }

        if (statusPrev.objWeighted_ != statusBestFound.objWeighted_) {
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
            independent_analysis_.UpdateStatus(jobOrderRef,
                                               statusPrev.startTimeVector_);
        }

#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
        return findBetterJobOrderWithinIterations;
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

    bool BreakFinishPermutation(double &accumLengthMin, JobCEC jobRelocate,
                                LLint startP, LLint finishP,
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

        IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(
            dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions,
            startTimeVector, processorJobVec, schedulable);
        countIterationStatus++;

#ifdef CHECK_IA_CORRECTNESS
        if (if_IA_skip == true) {
            double objCurr = ObjectiveFunctionBase::TrueObj(
                dagTasks, tasksInfo, startTimeVector, scheduleOptions);
            double objPrev = ObjectiveFunctionBase::TrueObj(
                dagTasks, tasksInfo, statusPrev.startTimeVector_,
                scheduleOptions);
            if (objCurr + 1e-3 < objPrev) {
                std::vector<uint> processorJobVecOld;
                OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions,
                                         jobOrderRef, processorJobVecOld,
                                         ObjectiveFunctionBase::type_trait);
                if (CompareVector(processorJobVec, processorJobVecOld)) {
                    jobOrderRef.print();
                    std::cout << "\n" << statusPrev.startTimeVector_ << "\n\n";
                    PrintSchedule(tasksInfo, statusPrev.startTimeVector_);
                    std::cout << "\n\n";
                    PrintSchedule(tasksInfo, startTimeVector);
                    jobOrderCurrForFinish.print();
                    CoutWarning(
                        "Find a case where enableIndependentAnalysis fails!");
                }
            }
        }
#endif

        if (MakeProgress<OrderScheduler>(statusBestFound, statusCurr)) {
            // SFOrder jobOrderReCreate(tasksInfo, startTimeVector);
            // if (jobOrderReCreate != jobOrderCurrForFinish)
            //   jobOrderCurrForFinish = jobOrderReCreate; // Issue:
            //   startTimeVector is not optimal w.r.t.
            // jobOrderRecreate, which makes the following iteation fail

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

        independent_analysis_ = IndependentAnalysis(
            dagTasks, tasksInfo, jobOrderRef, statusPrev.startTimeVector_,
            scheduleOptions.processorNum_, ObjectiveFunctionBase::type_trait);
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
    bool if_IA_skip = false;
    IndependentAnalysis independent_analysis_;
};  // class DAGScheduleOptimizer

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel(
    DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit) {
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
