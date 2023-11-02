#pragma once
#include "sources/Optimization/OptimizeSFOrder_TOM.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// #define CHECK_IA_CORRECTNESS

template <typename OrderScheduler, typename ObjectiveFunctionBase>
class DAGScheduleOptimizer_Threshold
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

    DAGScheduleOptimizer_Threshold() {}

    DAGScheduleOptimizer_Threshold(
        const DAG_Model &dagInput, const ScheduleOptions &scheduleOptions,
        double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT)
        : DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>(
              dagInput, scheduleOptions, timeLimits) {}

    // Compare against statusPrev built from jobOrderRef, and update statusPrev
    // and jobOrderRef if success and return true
    bool CompareAndUpdateStatus(
        SFOrder &jobOrderCurrForFinish,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
        SFOrder &jobOrderBestFound) 
        override{
        VectorDynamic startTimeVector;
        std::vector<uint> processorJobVec;
        startTimeVector = SimpleOrderScheduler::schedule(
            dagTasks, tasksInfo, scheduleOptions, jobOrderCurrForFinish,
            processorJobVec, ObjectiveFunctionBase::type_trait);
        double simple_scheduler_obj = ObjectiveFunctionBase::Evaluate(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
        double simple_threshold = 0;
        for (auto &chain: dagTasks.chains_) {
            simple_threshold += dagTasks.tasks[chain[0]].period - dagTasks.tasks[chain[0]].executionTime;
        }
        if (simple_scheduler_obj - simple_threshold > statusBestFound.objWeighted_)
            return false;

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

        return Base::WhetherMakeProgressAndUpdateBestFoundStatus(
            startTimeVector, processorJobVec, schedulable,
            jobOrderCurrForFinish, statusBestFound, jobOrderBestFound);
    }

};  // class DAGScheduleOptimizer_Threshold

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel_Threshold(
    DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
    CheckValidDAGTaskSetGivenObjType(dagTasks,
                                     ObjectiveFunctionBase::type_trait);
    DAGScheduleOptimizer_Threshold<OrderScheduler, ObjectiveFunctionBase>
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
