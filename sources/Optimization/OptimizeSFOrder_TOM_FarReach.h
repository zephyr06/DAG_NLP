#pragma once
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/OrderScheduler.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// #define CHECK_IA_CORRECTNESS

template <typename OrderScheduler, typename ObjectiveFunctionBase>
class DAGScheduleOptimizer_FarReach
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

    DAGScheduleOptimizer_FarReach() {}

    DAGScheduleOptimizer_FarReach(
        const DAG_Model& dagInput, const ScheduleOptions& scheduleOptions,
        double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT)
        : DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase>(
            dagInput, scheduleOptions, timeLimits) {
    }

    void CheckLPConsistency(
        SFOrder& jobOrderRefNew,
        IterationStatus<OrderScheduler, ObjectiveFunctionBase>& statusBestFound,
        SFOrder& jobOrderBestFound) override{
        return; // do nothing, TOM_FarReach can break the job order constraints
    }

}; // class DAGScheduleOptimizer_FarReach

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel_FarReach(
    DAG_Model& dagTasks, const ScheduleOptions& scheduleOptions,
    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
    CheckValidDAGTaskSetGivenObjType(dagTasks,
        ObjectiveFunctionBase::type_trait);
    // Force change to flex job order LP
    DAGScheduleOptimizer_FarReach<LPOrderSchedulerFlexJobOrder, ObjectiveFunctionBase>
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
