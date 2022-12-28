#pragma once
#include "unordered_map"
#include "unordered_set"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/ProcessorAssignment.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/SFOrderLPOptimizer.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
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
};

class SimpleOrderScheduler : public OrderScheduler {
public:
  // processorJobVec will be assigned values
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {
    return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                             processorJobVec);
  }

  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder) {
    return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder);
  }
};
// TODO: this function is wrong, fix it!
class LPOrderScheduler : public OrderScheduler {
public:
  static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder,
                                std::vector<uint> &processorJobVec) {
    if (!ProcessorAssignment::AssignProcessor(tasksInfo, jobOrder, scheduleOptions.processorNum_, processorJobVec))
    { // SFOrder unschedulable
      VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
      startTimeVector(0) = -1;
      processorJobVec.resize(tasksInfo.variableDimension, -1);
      return startTimeVector;
    }
    SFOrderLPOptimizer sfOrderLPOptimizer(dagTasks, jobOrder);
    sfOrderLPOptimizer.Optimize(processorJobVec);
    VectorDynamic startTimeVectorOptmized = sfOrderLPOptimizer.getOptimizedStartTimeVector();
    jobOrder = SFOrder(tasksInfo, startTimeVectorOptmized);
    return startTimeVectorOptmized;
    // ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer(dagTasks);
    // auto stv = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
    //                              processorJobVec);
    // scheduleOptimizer.setObjType(scheduleOptions.considerSensorFusion_);
    // scheduleOptimizer.Optimize(stv, processorJobVec);
    // return scheduleOptimizer.getOptimizedStartTimeVector();
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