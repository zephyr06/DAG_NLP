#pragma once
#include <algorithm> // for copy() and assign()
#include <iterator>  // for back_inserter

#include "sources/Optimization/InitialEstimate.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/Interval.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ProcessorAssignment.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/SkipUnschedulablePermutations.h"
#include "sources/Utils/OptimizeOrderUtils.h"
// #include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// extern int infeasibleCount;

std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks);
JobGroupRange FindJobActivateRange(const JobCEC &jobRelocate, SFOrder &jobOrderRef,
                                   const TaskSetInfoDerived &tasksInfo);

enum SFOrderCompareStatus { Infeasible, InferiorFeasible, BetterFeasible };
template <typename OrderScheduler, typename ObjectiveFunctionBase> class DAGScheduleOptimizer {
public:
  DAGScheduleOptimizer() {}

  DAGScheduleOptimizer(const DAG_Model &dagInput, const ScheduleOptions &scheduleOptions,
                       double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit)
      : start_time(std::chrono::system_clock::now()),
        timeLimits(GlobalVariablesDAGOpt::makeProgressTimeLimit), dagTasks(dagInput),
        tasksInfo(TaskSetInfoDerived(dagTasks.tasks)), scheduleOptions(scheduleOptions) {
    if (dagTasks.chains_.size() == 0)
      CoutWarning("No chain is provided for the given dag!");

    VectorDynamic initialSTV;
    // TODO: move to arguments
    if (scheduleOptions.selectInitialFromPoolCandidate_ != 0)
      initialSTV = SelectInitialFromPool<ObjectiveFunctionBase>(dagTasks, tasksInfo, scheduleOptions);
    else
      initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);

    jobOrderRef = SFOrder(tasksInfo, initialSTV);
    statusPrev = IterationStatus<OrderScheduler, ObjectiveFunctionBase>(dagTasks, tasksInfo, jobOrderRef,
                                                                        scheduleOptions);
    warmStart_ = statusPrev.startTimeVector_;
    // TODO: SelectInitialFromPool doesn't work well for simple order scheduler because it may leads into
    // unschedulable results

    if (!statusPrev.schedulable_)
      CoutWarning("Initial schedule is not schedulable!!!");

    if (GlobalVariablesDAGOpt::debugMode == 1) {
      std::cout << "Initial schedule: " << std::endl;
      PrintSchedule(tasksInfo, initialSTV);
      std::cout << initialSTV << std::endl;
      std::cout << "Initial SF order: " << std::endl;
      jobOrderRef.print();
    }
  }

  ScheduleResult Optimize() {
    BeginTimer("OptimizeDAG");
    while (ifContinue()) {
      countOutermostWhileLoop++;
      if (GlobalVariablesDAGOpt::debugMode == 1)
        std::cout << "Outer loop count: " << countOutermostWhileLoop << std::endl;
      findBetterJobOrderWithinIterations = false; // iterations stop unless a better job order is found

      // search the tasks related to task chain at first
      std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
      for (int i : taskIdSet) {
        for (LLint j = 0; j < 0 + tasksInfo.sizeOfVariables[i] && (ifContinue()); j++) {
          JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);
          ImproveJobOrderPerJob(jobRelocate);
        }
      }

      if (!findBetterJobOrderWithinIterations)
        break;
    }

    std::vector<uint> processorJobVec;
    auto stv = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrderRef, processorJobVec);
    ScheduleResult scheduleRes{
        jobOrderRef, statusPrev.startTimeVector_, statusPrev.schedulable_,
        ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, statusPrev.startTimeVector_, scheduleOptions),
        processorJobVec};
    EndTimer("OptimizeDAG");
    return scheduleRes;
  }

  inline SFOrder GetJobOrder() const { return jobOrderRef; }

  bool ifTimeout() const {
    auto curr_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= timeLimits) {
      std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << timeLimits << " seconds.\n\n";
      return true;
    }
    return false;
  }

  inline bool ifOptimal() const { return statusPrev.objWeighted_ == 0; }

  inline bool ifContinue() const { return (!ifTimeout()) && (!ifOptimal()); }

  bool ImproveJobOrderPerJob(const JobCEC &jobRelocate) {
    BeginTimer("ImproveJobOrderPerJob");
    JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, jobOrderRef, tasksInfo);

    IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusBestFound = statusPrev;
    SFOrder jobOrderBestFound = jobOrderRef;
    SFOrder jobOrderCurrForStart = jobOrderRef;
    jobOrderCurrForStart.RemoveJob(jobRelocate);

    for (LLint startP = jobStartFinishInstActiveRange.minIndex;
         startP < jobStartFinishInstActiveRange.maxIndex && ifContinue(); startP++) {

      if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurrForStart))
        continue;

      jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
      double accumLengthMin = 0;

      warmStart_(0) = -1;
      bool examJobOrderSchedulabilityOnce = false;
      for (LLint finishP = startP + 1; finishP < jobStartFinishInstActiveRange.maxIndex + 1 && ifContinue();
           finishP++) {
        if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, jobOrderRef))
          continue;
        if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP, tasksInfo, jobOrderCurrForStart,
                                      startP))
          break;

        SFOrder jobOrderCurrForFinish = jobOrderCurrForStart; // strangely, copying by value is faster
        jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);

        // bool debug_infeasible = false;
        if (!examJobOrderSchedulabilityOnce) {
          std::vector<uint> processorJobVec;
          if ((!ProcessorAssignment::AssignProcessor(tasksInfo, jobOrderCurrForFinish,
                                                     scheduleOptions.processorNum_, processorJobVec))) {

            jobOrderCurrForFinish.RemoveInstance(jobRelocate, finishP);
            break;
            // debug_infeasible = true;
          }
          examJobOrderSchedulabilityOnce = true;
        }
        CompareAndUpdateStatus(jobOrderCurrForFinish, jobStartFinishInstActiveRange, statusBestFound,
                               jobOrderBestFound);

        // TODO: whether it's possible to avoid whetherSFMapNeedUpdate
        // bool s = jobOrderCurrForFinish.whetherSFMapNeedUpdate;
        jobOrderCurrForFinish.RemoveInstance(jobRelocate, finishP);
        // jobOrderCurrForFinish.whetherSFMapNeedUpdate = s;
      }
      jobOrderCurrForStart.RemoveInstance(jobRelocate, startP);
    }

    if (statusPrev.objWeighted_ != statusBestFound.objWeighted_) {
      statusPrev = statusBestFound;
      jobOrderRef = jobOrderBestFound;
      findBetterJobOrderWithinIterations = true;
      countMakeProgress++;
    }

    EndTimer("ImproveJobOrderPerJob");
    return findBetterJobOrderWithinIterations;
  }

  // Compare against statusPrev built from jobOrderRef, and update statusPrev and jobOrderRef if success and
  // return true
  // TODO: jobOrderCurrForFinish may become different after optimization
  // TODO: SubGroupSchedulabilityCheck is temporarily removed, which is used to check whether the small job
  // order under influence is unschedulable
  bool CompareAndUpdateStatus(SFOrder &jobOrderCurrForFinish, JobGroupRange &jobStartFinishInstActiveRange,
                              IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusBestFound,
                              SFOrder &jobOrderBestFound) {
    VectorDynamic startTimeVector;
    std::vector<uint> processorJobVec;
    startTimeVector = OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrderCurrForFinish,
                                               processorJobVec, warmStart_);
    warmStart_ = startTimeVector;
    bool schedulable = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector, processorJobVec,
                                             scheduleOptions.processorNum_);
    if (!schedulable) {
      infeasibleCount++;
      return false;
    }

    IterationStatus<OrderScheduler, ObjectiveFunctionBase> statusCurr(
        dagTasks, tasksInfo, jobOrderCurrForFinish, scheduleOptions, startTimeVector, processorJobVec,
        schedulable);
    countIterationStatus++;

    if (MakeProgress<OrderScheduler>(statusBestFound, statusCurr)) {
      statusBestFound = statusCurr;
      jobOrderBestFound = jobOrderCurrForFinish;
      if (GlobalVariablesDAGOpt::debugMode == 1) {
        std::cout << "Make progress!" << std::endl;
        std::cout << "start time vector: \n" << statusPrev.startTimeVector_ << "\n";
        PrintSchedule(tasksInfo, statusPrev.startTimeVector_);
      }
      return true;
    } else
      return false;
  }

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
  VectorDynamic warmStart_;

}; // class DAGScheduleOptimizer

template <typename OrderScheduler, typename ObjectiveFunctionBase>
ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions,
                                double timeLimits = GlobalVariablesDAGOpt::makeProgressTimeLimit) {
  DAGScheduleOptimizer<OrderScheduler, ObjectiveFunctionBase> dagScheduleOptimizer(dagTasks, scheduleOptions,
                                                                                   timeLimits);
  const TaskSetInfoDerived &tasksInfo = dagScheduleOptimizer.tasksInfo;
  const SFOrder &jobOrderRef = dagScheduleOptimizer.GetJobOrder();
  ScheduleResult scheduleRes = dagScheduleOptimizer.Optimize();

  // TODO(Dong) : optimize this part
  if (scheduleOptions.doScheduleOptimization_) {
    if (!scheduleOptions.considerSensorFusion_ || !scheduleRes.schedulable_) {
      ScheduleOptimizer schedule_optimizer = ScheduleOptimizer(dagTasks);
      if (scheduleOptions.considerSensorFusion_) {
        // TODO(Dong): modify related code
        // schedule_optimizer.OptimizeObjWeighted(dagTasks, scheduleRes);
        // ScheduleResult result_after_optimization;
        // result_after_optimization = schedule_optimizer.getOptimizedResult();
        // if (result_after_optimization.objWeighted_ < scheduleRes.objWeighted_)
        // {
        //     scheduleRes = result_after_optimization;
        //     std::vector<RTDA> rtda_vector;
        //     for (auto chain : dagTasks.chains_)
        //     {
        //         auto res = GetRTDAFromSingleJob(tasksInfo, chain, scheduleRes.startTimeVector_);
        //         RTDA resM = GetMaxRTDA(res);
        //         rtda_vector.push_back(resM);
        //     }
        //     scheduleRes.obj_ = ObjRTDA(rtda_vector);
        // }
      } else {
        schedule_optimizer.setObjType(false);
        schedule_optimizer.Optimize(scheduleRes.startTimeVector_, scheduleRes.processorJobVec_);
        scheduleRes.startTimeVector_ = schedule_optimizer.getOptimizedStartTimeVector();
        scheduleRes.obj_ = ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, scheduleRes.startTimeVector_,
                                                          scheduleOptions);
      }
    }
  }

  if (ObjectiveFunctionBase::type_trait == "RTDAExperimentObj")
    scheduleRes.schedulable_ =
        ExamBasic_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_,
                              scheduleOptions.processorNum_);
  else if (ObjectiveFunctionBase::type_trait == "RTSS21ICObj")
    scheduleRes.schedulable_ =
        ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_,
                            scheduleOptions.processorNum_, GlobalVariablesDAGOpt::sensorFusionTolerance,
                            GlobalVariablesDAGOpt::freshTol);
  else
    scheduleRes.schedulable_ = false;
  std::cout << "Outermost while loop count: " << dagScheduleOptimizer.countOutermostWhileLoop << std::endl;
  std::cout << "Make progress count: " << dagScheduleOptimizer.countMakeProgress << std::endl;
  std::cout << Color::blue
            << "Candidate Iteration Status count: " << dagScheduleOptimizer.countIterationStatus << Color::def
            << std::endl;
  std::cout << "infeasibleCount: " << dagScheduleOptimizer.infeasibleCount << std::endl;
  std::cout << "Total number of variables: " << tasksInfo.length << std::endl;
  scheduleRes.countOutermostWhileLoop_ = dagScheduleOptimizer.countOutermostWhileLoop;
  scheduleRes.countMakeProgress_ = dagScheduleOptimizer.countMakeProgress;
  scheduleRes.countIterationStatus_ = dagScheduleOptimizer.countIterationStatus;

  return scheduleRes;
}
} // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
