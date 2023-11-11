#pragma once
#include "sources/Optimization/OptimizeSFOrder_TOM.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// #define CHECK_IA_CORRECTNESS

template <typename OrderScheduler, typename ObjectiveFunctionBase>
class DAGScheduleOptimizer_Raw
    : public DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>
{
public:
    // bring variables from the base class
    typedef DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase> Base;
    using Base::dagTasks; // bring members from base class
    using Base::findBetterJobOrderWithinIterations;
    using Base::ifContinue;
    using Base::jobOrderRef;
    using Base::scheduleOptions;
    using Base::statusPrev;
    using Base::tasksInfo;
    using Base::CompareAndUpdateStatus;
    using Base::ApplyBestAdjacentJobOrderOfOneJob;

    DAGScheduleOptimizer_Raw() {}

    DAGScheduleOptimizer_Raw(
        const DAG_Model& dagInput, const ScheduleOptions& scheduleOptions,
        double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT)
        : DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>(
            dagInput, scheduleOptions, timeLimits) {
    }

    void ImproveJobOrderPerJob(const JobCEC& jobRelocate) override {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif

        JobGroupRange jobIndexRange(0, static_cast<int>(tasksInfo.length) * 2 - 2);

        IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusBestFound = statusPrev;
        jobOrderRef.EstablishJobSFMap();
        SFOrder jobOrderBestFound = jobOrderRef;

        for (LLint startP = jobIndexRange.minIndex;
            startP <= jobIndexRange.maxIndex - 2 && startP <= int(tasksInfo.length) * 2 - 2 && ifContinue();
            startP++) {
            SFOrder jobOrderCurrForStart = jobOrderRef;
            jobOrderCurrForStart.RemoveJob(jobRelocate);

            jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
            for (LLint finishP = startP + 1;
                finishP <= std::min(jobIndexRange.maxIndex - 1,
                    static_cast<int>(tasksInfo.length) * 2 - 1) && ifContinue();
                finishP++) {

                SFOrder jobOrderCurrForFinish = jobOrderCurrForStart; //  copying by value is sometimes
                //  faster
                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

                CompareAndUpdateStatus(jobOrderCurrForFinish, statusBestFound,
                    jobOrderBestFound);
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

}; // class DAGScheduleOptimizer_Raw

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel_Raw(
    DAG_Model& dagTasks, const ScheduleOptions& scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
    CheckValidDAGTaskSetGivenObjType(dagTasks,
        ObjectiveFunctionBase::type_trait);
    DAGScheduleOptimizer_Raw<OrderScheduler, ObjectiveFunctionBase>
        dagScheduleOptimizer(dagTasks, scheduleOptions, timeLimits);
    const TaskSetInfoDerived& tasksInfo = dagScheduleOptimizer.tasksInfo;
    const SFOrder& jobOrderRef = dagScheduleOptimizer.GetJobOrder();
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
} // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
