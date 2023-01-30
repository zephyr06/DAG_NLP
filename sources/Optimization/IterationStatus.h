#pragma once

#include "sources/Optimization/InitialEstimate.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/Interval.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {

// extern int infeasibleCount;

template <typename OrderScheduler, typename ObjectiveFunctionBase> class IterationStatus {
public:
  bool schedulable_; // only basic schedulability
  double objWeighted_;
  // SFOrder jobOrder_;
  VectorDynamic startTimeVector_;

  IterationStatus() : schedulable_(false), objWeighted_(1e9), startTimeVector_(GenerateVectorDynamic1D(0)) {}

  IterationStatus(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                  const ScheduleOptions &scheduleOptions) {
    // BeginTimer(__FUNCTION__);
    std::vector<uint> processorJobVec;

    // jobOrder_ = jobOrder;
    startTimeVector_ =
        OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
    schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec,
                                         scheduleOptions.processorNum_);
    if (!schedulable_)
      objWeighted_ = 1e9;
    else
      objWeighted_ = ObjectiveFunctionBase::Evaluate(dagTasks, tasksInfo, startTimeVector_, scheduleOptions);
    // EndTimer(__FUNCTION__);
  }

  IterationStatus(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                  const ScheduleOptions &scheduleOptions, VectorDynamic startTimeVector,
                  std::vector<uint> processorJobVec, bool schedulable)
      : schedulable_(schedulable), startTimeVector_(startTimeVector) {
    // BeginTimer(__FUNCTION__);
    if (!schedulable_)
      objWeighted_ = 1e9;
    else
      objWeighted_ = ObjectiveFunctionBase::Evaluate(dagTasks, tasksInfo, startTimeVector_, scheduleOptions);
    // EndTimer(__FUNCTION__);
  }
};

template <typename OrderScheduler, typename ObjectiveFunctionBase>
bool MakeProgress(const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusPrev,
                  const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusCurr) {
  if (!statusCurr.schedulable_) {
    // infeasibleCount++;
    // if (GlobalVariablesDAGOpt::debugMode == 1)
    // {
    //     // TaskSetInfoDerived tasksInfo(statusCurr.dagTasks_.tasks);
    //     std::cout << "Infeasible schedule #:" << infeasibleCount << std::endl;
    //     // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
    //     // statusCurr.jobOrder.print();
    // }
    return false;
  }
  if (statusCurr.objWeighted_ < statusPrev.objWeighted_)
    return true;
  return false;
}
} // namespace OptimizeSF

} // namespace OrderOptDAG_SPACE