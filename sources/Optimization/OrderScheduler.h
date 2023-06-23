#pragma once
#include "unordered_map"
#include "unordered_set"

// #include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/ProcessorAssignment.h"
#include "sources/Optimization/SFOrderLPOptimizer.h"
// #include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/profilier.h"
// #include "sources/Optimization/InitialEstimate.h"

namespace OrderOptDAG_SPACE {
class OrderScheduler {
   public:
    // jobOrder is allowed to change
    // processorJobVec will be assigned values
    OrderScheduler() {}
    static VectorDynamic schedule(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder) {
        CoutError("Never call base function!");
        return GenerateVectorDynamic1D(0);
    }
    static VectorDynamic schedule(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
        std::vector<uint> &processorJobVec, std::string obj_type = "none") {
        CoutError("Never call base function!");
        return GenerateVectorDynamic1D(0);
    }
};

class SimpleOrderScheduler : public OrderScheduler {
   public:
    static VectorDynamic schedule(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder) {
        auto stv = SFOrderScheduling(dagTasks.tasks, tasksInfo,
                                     scheduleOptions.processorNum_, jobOrder);
        SFOrder jobOrderNew = SFOrder(tasksInfo, stv);
        if (jobOrderNew != jobOrder) {
            return GetInfeasibleStartTimeVector(tasksInfo);
        }
        return stv;
    }
    // processorJobVec will be assigned values
    static VectorDynamic schedule(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
        std::vector<uint> &processorJobVec, std::string obj_type = "none") {
        auto stv = SFOrderScheduling(dagTasks.tasks, tasksInfo,
                                     scheduleOptions.processorNum_, jobOrder,
                                     processorJobVec);
        SFOrder jobOrderNew = SFOrder(tasksInfo, stv);
        if (jobOrderNew != jobOrder) {
            return GetInfeasibleStartTimeVector(tasksInfo);
        }
        return stv;
    }
};

// TODO: test all the components becuse LP perrforms much worse than simple in
// n15_v1, RTDA objective function
class LPOrderScheduler : public OrderScheduler {
   public:
    // TODO: add setStart() as an advanced initialization
    static VectorDynamic schedule(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
        std::vector<uint> &processorJobVec, std::string obj_type = "none") {
#ifdef PROFILE_CODE
        BeginTimer("LPOrderScheduler_schedule");
#endif
        if (!ProcessorAssignment::AssignProcessor(
                tasksInfo, jobOrder, scheduleOptions.processorNum_,
                processorJobVec)) {  // SFOrder unschedulable
            VectorDynamic startTimeVector =
                GenerateVectorDynamic(tasksInfo.variableDimension);
            startTimeVector(0) = -1;
            // assign all jobs to processor 0 to avoid errors in codes, this
            // will not affect the correctness.
            processorJobVec.resize(tasksInfo.variableDimension, 0);
#ifdef PROFILE_CODE
            EndTimer("LPOrderScheduler_schedule");
#endif
            return startTimeVector;
        }

        SFOrderLPOptimizer sfOrderLPOptimizer(
            dagTasks, jobOrder, scheduleOptions.processorNum_, obj_type);
        sfOrderLPOptimizer.Optimize(processorJobVec);
        VectorDynamic startTimeVectorOptmized =
            sfOrderLPOptimizer.getOptimizedStartTimeVector();
        // TODO: need to check carefully whether the code in
        // `OptimizeSFOrder.cpp/.h` support changing joborder? jobOrder =
        // SFOrder(tasksInfo, startTimeVectorOptmized);
#ifdef PROFILE_CODE
        EndTimer("LPOrderScheduler_schedule");
#endif
        return startTimeVectorOptmized;
    }
};

}  // namespace OrderOptDAG_SPACE