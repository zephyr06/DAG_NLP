
#include "sources/Optimization/GlobalOptimization.h"

namespace OrderOptDAG_SPACE {

// void PrintTimeSequence2D(const std::vector<std::vector<TimeInstance>> &results) {
//   for (uint i = 0; i < results.size(); i++) {
//     std::cout << "Chain " << i << ": \n";
//     for (uint j = 0; j < results[i].size(); j++) {
//       std::cout << "(" << results[i][j].type << ", " << results[i][j].job.taskId << ", "
//                 << results[i][j].job.jobId << "), "
//                 << "\n";
//     }
//   }
// }

void IterateTimeInstanceSeq(std::vector<TimeInstance> &prevSeq, std::vector<JobQueueOfATask> &jobQueueTaskSet,
                            PermutationStatus &permutationStatus) {
  if (permutationStatus.ifTimeout()) {
    permutationStatus.whetherTimeOut = true;
    return;
  }
  // if all reachEnd, push into result;
  bool whetherAllReachEnd = true;
  for (uint i = 0; i < jobQueueTaskSet.size(); i++) {
    if (!jobQueueTaskSet[i].ReachEnd()) {
      whetherAllReachEnd = false;
      break;
    }
  }
  if (whetherAllReachEnd) {
    SFOrder jobOrder(permutationStatus.tasksInfo, prevSeq);
    permutationStatus.TryUpdate(jobOrder);
    return;
  }

  for (uint i = 0; i < jobQueueTaskSet.size(); i++) {
    if (jobQueueTaskSet[i].ReachEnd())
      continue;
    TimeInstance instCurr = jobQueueTaskSet[i].GetTimeInstance();
    if (prevSeq.size() > 0) {
      if (instCurr.GetRangeMax(permutationStatus.tasksInfo) <
          prevSeq.back().GetRangeMin(permutationStatus.tasksInfo))
        continue;
      // ;
    }
    prevSeq.push_back(instCurr);
    jobQueueTaskSet[i].MoveForward();
    IterateTimeInstanceSeq(prevSeq, jobQueueTaskSet, permutationStatus);
    jobQueueTaskSet[i].MoveBackward();
    prevSeq.pop_back();
  }
}

PermutationStatus FindGlobalOptRTDA(const DAG_Model &dagTasks, const ScheduleOptions &scheduleOptions) {
  TaskSetInfoDerived tasksInfo(dagTasks.tasks);
  return FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
}

PermutationStatus FindGlobalOptRTDA(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                    const ScheduleOptions &scheduleOptions) {
  int N = dagTasks.tasks.size();
  if (N > 3)
    CoutError("FindAllJobOrderPermutations only works with task sets of 3 DAGs!");

  std::vector<JobQueueOfATask> jobQueueTaskSet;
  jobQueueTaskSet.reserve(N);
  for (int i = 0; i < N; i++)
    jobQueueTaskSet.push_back(
        JobQueueOfATask(dagTasks.tasks[i], tasksInfo.hyper_period / dagTasks.tasks[i].period));

  std::vector<TimeInstance> prevSeq;
  prevSeq.reserve(2 * tasksInfo.length);

  PermutationStatus permutationStatus(dagTasks, tasksInfo, scheduleOptions);
  IterateTimeInstanceSeq(prevSeq, jobQueueTaskSet, permutationStatus);
  return permutationStatus;
}
} // namespace OrderOptDAG_SPACE