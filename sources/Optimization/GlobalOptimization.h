#pragma once
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/OptimizeOrderUtils.h"

namespace OrderOptDAG_SPACE {
using namespace OptimizeSF;

class JobQueueOfATask {
public:
  Task task_;
  size_t jobIndexCurr;
  size_t maxJobNum;
  JobQueueOfATask(const Task &task, int maxJobNum) : task_(task), jobIndexCurr(0), maxJobNum(maxJobNum) {}

  bool ReachEnd() const { return jobIndexCurr >= maxJobNum * 2; }
  bool ReachBegin() const { return jobIndexCurr <= 0; }
  bool MoveForward() {
    if (ReachEnd())
      return false;
    jobIndexCurr++;
    return true;
  }
  bool MoveBackward() {
    if (ReachBegin())
      return false;
    jobIndexCurr--;
    return true;
  }
  TimeInstance GetTimeInstance() const {
    char type = 's';
    if (jobIndexCurr % 2 == 1)
      type = 'f';
    return TimeInstance(type, JobCEC(task_.id, jobIndexCurr / 2));
  }
};

class PermutationStatus {
public:
  typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimerType;

  DAG_Model dagTasks;
  TaskSetInfoDerived tasksInfo;
  ScheduleOptions scheduleOptions;
  SFOrder orderOpt_;
  int totalPermutation_ = 0;
  double valOpt_ = 1e90;
  TimerType startTime_;
  double timeLimits_ = GlobalVariablesDAGOpt::kGlobalOptimizationTimeLimit;
  bool whetherTimeOut = false;

  PermutationStatus(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                    const ScheduleOptions &scheduleOptions)
      : dagTasks(dagTasks), tasksInfo(tasksInfo), scheduleOptions(scheduleOptions),
        startTime_(std::chrono::high_resolution_clock::now()) {}

  void TryUpdate(SFOrder jobOrder) {

    totalPermutation_++;

    std::vector<uint> processorJobVec;
    VectorDynamic startTimeOpt =
        LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
    bool schedulable = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeOpt, processorJobVec,
                                             scheduleOptions.processorNum_);
    // jobOrder.print();

    if (!schedulable)
      return;

    double evalCurr = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeOpt, scheduleOptions);
#ifdef SAVE_ITERATION_STAT
    AppendResultsToFile(evalCurr);
#endif
    // std::cout << "startTimeOpt: " << startTimeOpt << "\n\n";
    if (evalCurr < valOpt_) {
      std::cout << "Find a better permutation with value: " << evalCurr << "\n";
      valOpt_ = evalCurr;
      orderOpt_ = jobOrder;
    }
  }

  void print() const {
    std::cout << "Global optimal job order found is: \n";
    orderOpt_.print();
    std::cout << "\n";
    std::cout << "Global optimal RTDA found is: \n" << valOpt_ << "\n";
    std::cout << "Total number of permutations iterated is: \n" << totalPermutation_ << "\n";
  }

  ScheduleResult GetScheduleResult() {
    if (whetherTimeOut) {
      return ScheduleResult(orderOpt_, GenerateVectorDynamic1D(0), false, valOpt_);
    }
    std::vector<uint> processorJobVec;
    VectorDynamic startTimeOpt =
        LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, orderOpt_, processorJobVec);
    bool schedulable = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeOpt, processorJobVec,
                                             scheduleOptions.processorNum_);
    return ScheduleResult(orderOpt_, startTimeOpt, schedulable, valOpt_);
  }

  bool ifTimeout() const {
    auto curr_time = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - startTime_).count() >= timeLimits_) {
      std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << timeLimits_
                << " seconds.\n\n";
      return true;
    }
    return false;
  }
};

// void PrintTimeSequence2D(const std::vector<std::vector<TimeInstance>> &results);

void IterateTimeInstanceSeq(std::vector<TimeInstance> &prevSeq, std::vector<JobQueueOfATask> &jobQueueTaskSet,
                            PermutationStatus &permutationStatus);

PermutationStatus FindGlobalOptRTDA(const DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions);

PermutationStatus FindGlobalOptRTDA(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                    const ScheduleOptions &scheduleOptions);

} // namespace OrderOptDAG_SPACE