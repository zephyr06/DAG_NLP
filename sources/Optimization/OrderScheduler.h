#pragma once
#include "unordered_map"
#include "unordered_set"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Optimization/LinearProgrammingSolver.h"
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
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {
    CoutError("Never call base function!");
    return GenerateVectorDynamic1D(0);
  }

  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder) {
    CoutError("Never call base function!");
    return GenerateVectorDynamic1D(0);
  }

  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                const VectorDynamic &warmStart) {
    CoutError("Never call base function!");
    return GenerateVectorDynamic1D(0);
  }
};

class SimpleOrderScheduler : public OrderScheduler {
public:
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder) {
    return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder);
  }
  // processorJobVec will be assigned values
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {
    return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                             processorJobVec);
  }

  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec, const VectorDynamic &warmStart) {
    return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                             processorJobVec);
  }
};
// TODO: test all the components becuse LP perrforms much worse than simple in n15_v1, RTDA objective function
class LPOrderScheduler : public OrderScheduler {
public:
  // TODO: add setStart() as an advanced initialization
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {

#ifdef PROFILE_CODE
    BeginTimer("LPOrderScheduler_schedule");
#endif
    if (!ProcessorAssignment::AssignProcessor(tasksInfo, jobOrder, scheduleOptions.processorNum_,
                                              processorJobVec)) { // SFOrder unschedulable
      VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
      startTimeVector(0) = -1;
      // assign all jobs to processor 0 to avoid errors in codes, this will not affect the correctness.
      processorJobVec.resize(tasksInfo.variableDimension, 0);
#ifdef PROFILE_CODE
      EndTimer("LPOrderScheduler_schedule");
#endif

      return startTimeVector;
    }
    SFOrderLPOptimizer sfOrderLPOptimizer(dagTasks, jobOrder, scheduleOptions.processorNum_);
    sfOrderLPOptimizer.Optimize(processorJobVec);
    VectorDynamic startTimeVectorOptmized = sfOrderLPOptimizer.getOptimizedStartTimeVector();
// TODO: need to check carefully whether the code in `OptimizeSFOrder.cpp/.h` support changing joborder?
// jobOrder = SFOrder(tasksInfo, startTimeVectorOptmized);
#ifdef PROFILE_CODE
    EndTimer("LPOrderScheduler_schedule");
#endif
    return startTimeVectorOptmized;
  }
  // TODO: merge same code
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec, const VectorDynamic &warmStart) {
#ifdef PROFILE_CODE
    BeginTimer("LPOrderScheduler_schedule");
#endif
    if (!ProcessorAssignment::AssignProcessor(tasksInfo, jobOrder, scheduleOptions.processorNum_,
                                              processorJobVec)) { // SFOrder unschedulable
      VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
      startTimeVector(0) = -1;
      // assign all jobs to processor 0 to avoid errors in codes, this will not affect the correctness.
      processorJobVec.resize(tasksInfo.variableDimension, 0);
#ifdef PROFILE_CODE
      EndTimer("LPOrderScheduler_schedule");
#endif
      return startTimeVector;
    }
    SFOrderLPOptimizer sfOrderLPOptimizer(dagTasks, jobOrder, scheduleOptions.processorNum_);
    if (warmStart(0) != -1)
      sfOrderLPOptimizer.Optimize(processorJobVec, warmStart);
    else
      sfOrderLPOptimizer.Optimize(processorJobVec);
    VectorDynamic startTimeVectorOptmized = sfOrderLPOptimizer.getOptimizedStartTimeVector();
// TODO: need to check carefully whether the code in `OptimizeSFOrder.cpp/.h` support changing joborder?
// jobOrder = SFOrder(tasksInfo, startTimeVectorOptmized);
#ifdef PROFILE_CODE
    EndTimer("LPOrderScheduler_schedule");
#endif
    return startTimeVectorOptmized;
  }
};
// TODO: test !!
class IPMOrderScheduler : public OrderScheduler {
public:
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {

    bool whetherPA = ProcessorAssignment::AssignProcessor(tasksInfo, jobOrder, scheduleOptions.processorNum_,
                                                          processorJobVec);
    if (!whetherPA) {
      VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
      startTimeVector(0) = -1;
      return startTimeVector;
    }
    VectorDynamic xActual = OrderOptDAG_SPACE::OptRTDA_IPMOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                              scheduleOptions.processorNum_, true);

    if (eigen_is_nan(xActual)) {
      VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
      CoutWarning("Nan result found in IPMOrderScheduler!");
      startTimeVector(0) = -1;
      return startTimeVector;
    }
    return xActual;
  }
};
} // namespace OrderOptDAG_SPACE