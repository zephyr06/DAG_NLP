#pragma once
#include "sources/Optimization/OptimizeSFOrder_TOM.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// #define CHECK_IA_CORRECTNESS

template <typename OrderScheduler, typename ObjectiveFunctionBase>
class DAGScheduleOptimizer_IA
    : public DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase> {
   public:
    // bring variables from the base class
    typedef DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase> Base;
    using Base::dagTasks;  // bring members from base class
    using Base::findBetterJobOrderWithinIterations;
    using Base::ifContinue;
    using Base::jobOrderRef;
    using Base::scheduleOptions;
    using Base::statusPrev;
    using Base::tasksInfo;

    DAGScheduleOptimizer_IA() {}

    DAGScheduleOptimizer_IA(
        const DAG_Model &dagInput, const ScheduleOptions &scheduleOptions,
        double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT)
        : DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>(
              dagInput, scheduleOptions, timeLimits) {
        independent_analysis_ = IndependentAnalysis(
            dagTasks, tasksInfo, jobOrderRef, statusPrev.startTimeVector_,
            scheduleOptions.processorNum_, ObjectiveFunctionBase::type_trait);
    }

    void ImproveJobOrderPerJob(const JobCEC &jobRelocate) override{
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif

        if(independent_analysis_.WhetherSafeSkipFast(jobRelocate, jobOrderRef))
            return;
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
                // TODO: add WhetherJobInfluenceChainLength into IA?
                if_IA_skip = independent_analysis_.WhetherSafeSkip(
                    jobRelocate, startP, finishP, jobOrderRef);
#ifndef CHECK_IA_CORRECTNESS
                if (if_IA_skip)
                    continue;
#endif

                SFOrder jobOrderCurrForFinish =
                    jobOrderCurrForStart;  //  copying by value is sometimes
                                           //  faster
                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                if (Base::WhetherFinishPositionInfeasible(
                        accumLengthMin, jobRelocate, startP, finishP,
                        jobOrderCurrForStart, jobOrderCurrForFinish))
                    break;

                CompareAndUpdateStatus(jobOrderCurrForFinish, statusBestFound,
                                       jobOrderBestFound);
            }
            // jobOrderCurrForStart.RemoveStart(jobRelocate, startP);
        }

        if (statusPrev.objWeighted_ != statusBestFound.objWeighted_) {
            Base::ApplyBestAdjacentJobOrderOfOneJob(statusBestFound,
                                                    jobOrderBestFound);
            independent_analysis_.UpdateStatus(jobOrderRef,
                                               statusPrev.startTimeVector_);
        }

#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
        return;
    }

    // Compare against statusPrev built from jobOrderRef, and update statusPrev
    // and jobOrderRef if success and return true
    // TODO: SubGroupSchedulabilityCheck is temporarily removed, which is used
    // to check whether the small job order under influence is unschedulable
    bool CompareAndUpdateStatus(
        SFOrder &jobOrderCurrForFinish,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) override{
        VectorDynamic startTimeVector;
        std::vector<uint> processorJobVec;
        startTimeVector = OrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderCurrForFinish,
            processorJobVec, ObjectiveFunctionBase::type_trait);

        bool schedulable = ExamBasic_Feasibility(
            dagTasks, tasksInfo, startTimeVector, processorJobVec,
            scheduleOptions.processorNum_);
        if (!schedulable) {
            Base::infeasibleCount++;
            return false;
        }

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

        return Base::WhetherMakeProgressAndUpdateBestFoundStatus(
            startTimeVector, processorJobVec, schedulable,
            jobOrderCurrForFinish, statusBestFound, jobOrderBestFound);
    }

    // extra data members
    bool if_IA_skip = false;
    IndependentAnalysis independent_analysis_;
};  // class DAGScheduleOptimizer_IA

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel_IA(
    DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
    CheckValidDAGTaskSetGivenObjType(dagTasks,
                                     ObjectiveFunctionBase::type_trait);
    DAGScheduleOptimizer_IA<OrderScheduler, ObjectiveFunctionBase>
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
