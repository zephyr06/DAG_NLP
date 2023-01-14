#pragma once
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/TaskModel/DAG_Model.h"

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

// TODO: rename this function
void PrintTimeSequence2D(const std::vector<std::vector<TimeInstance>> &results);

void AddTimeInstance(std::vector<TimeInstance> &prevSeq, std::vector<JobQueueOfATask> &jobQueueTaskSet,
                     std::vector<std::vector<TimeInstance>> &results);

std::vector<std::vector<TimeInstance>> FindAllJobOrderPermutations(const DAG_Model &dagTasks,
                                                                   const TaskSetInfoDerived &tasksInfo);

double FindGlobalOptRTDA(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                         const ScheduleOptions &scheduleOptions);

} // namespace OrderOptDAG_SPACE