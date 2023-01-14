
#include "sources/Optimization/GlobalOptimization.h"

namespace OrderOptDAG_SPACE {

// TODO: rename this function
void PrintTimeSequence2D(const std::vector<std::vector<TimeInstance>> &results) {
  for (uint i = 0; i < results.size(); i++) {
    std::cout << "Chain " << i << ": \n";
    for (uint j = 0; j < results[i].size(); j++) {
      std::cout << "(" << results[i][j].type << ", " << results[i][j].job.taskId << ", "
                << results[i][j].job.jobId << "), "
                << "\n";
    }
  }
}

void AddTimeInstance(std::vector<TimeInstance> &prevSeq, std::vector<JobQueueOfATask> &jobQueueTaskSet,
                     std::vector<std::vector<TimeInstance>> &results) {
  // if all reachEnd, push into result;
  bool whetherAllReachEnd = true;
  for (uint i = 0; i < jobQueueTaskSet.size(); i++) {
    if (!jobQueueTaskSet[i].ReachEnd()) {
      whetherAllReachEnd = false;
      break;
    }
  }
  if (whetherAllReachEnd) {
    results.push_back(prevSeq);
    return;
  }

  for (uint i = 0; i < jobQueueTaskSet.size(); i++) {
    if (jobQueueTaskSet[i].ReachEnd())
      continue;
    prevSeq.push_back(jobQueueTaskSet[i].GetTimeInstance());
    jobQueueTaskSet[i].MoveForward();
    AddTimeInstance(prevSeq, jobQueueTaskSet, results);
    jobQueueTaskSet[i].MoveBackward();
    prevSeq.pop_back();
  }
}

std::vector<std::vector<TimeInstance>> FindAllJobOrderPermutations(const DAG_Model &dagTasks,
                                                                   const TaskSetInfoDerived &tasksInfo) {
  int N = dagTasks.tasks.size();
  if (N > 3)
    CoutError("FindAllJobOrderPermutations only works with task sets of 3 DAGs!");

  std::vector<JobQueueOfATask> jobQueueTaskSet;
  jobQueueTaskSet.reserve(N);
  for (int i = 0; i < N; i++)
    jobQueueTaskSet.push_back(
        JobQueueOfATask(dagTasks.tasks[i], tasksInfo.hyperPeriod / dagTasks.tasks[i].period));

  std::vector<std::vector<TimeInstance>> instOrderAll;
  instOrderAll.reserve(1000); // should be the max?
  std::vector<TimeInstance> prevSeq;
  prevSeq.reserve(2 * tasksInfo.length);
  AddTimeInstance(prevSeq, jobQueueTaskSet, instOrderAll);

  return instOrderAll;
}
std::vector<SFOrder> GetAllJobOrderPermutations(const DAG_Model &dagTasks,
                                                const TaskSetInfoDerived &tasksInfo) {
  std::vector<std::vector<TimeInstance>> instPermu = FindAllJobOrderPermutations(dagTasks, tasksInfo);
  std::vector<SFOrder> jobOrderAll;
  jobOrderAll.reserve(instPermu.size());
  for (uint i = 0; i < instPermu.size(); i++) {
    jobOrderAll.push_back(SFOrder(tasksInfo, instPermu[i]));
  }
  return jobOrderAll;
}

double FindGlobalOptRTDA(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                         const ScheduleOptions &scheduleOptions) {
  std::vector<SFOrder> jobOrderAll = GetAllJobOrderPermutations(dagTasks, tasksInfo);
  double globalOpt = 1e99;
  int globalOptIndex = -1;

  for (uint i = 0; i < jobOrderAll.size(); i++) {
    auto &jobOrder = jobOrderAll[i];
    std::vector<uint> processorJobVec;
    VectorDynamic startTimeOpt =
        LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
    bool schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeOpt, processorJobVec,
                                              scheduleOptions.processorNum_);
    if (!schedulable_)
      continue;

    double evalCurr = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeOpt, scheduleOptions);
    std::cout << "startTimeOpt: " << startTimeOpt << "\n\n";
    if (evalCurr < globalOpt) {
      std::cout << "Find a better permutation at the index: " << i << "\n";
      globalOpt = evalCurr;
      globalOptIndex = i;
    }
  }
  return globalOpt;
}
} // namespace OrderOptDAG_SPACE