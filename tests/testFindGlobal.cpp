#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "sources/Factors/JacobianAnalyze.h"
#include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/IncrementQR.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
using namespace LPOptimizer;
// TODO: Fix DDL issue, rhs should minus execution time;
class DAGScheduleOptimizerTest1 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v30.csv", "orig", 1);
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
    chain1 = {0, 2};
    dagTasks.chains_[0] = chain1;
    timeLimits = 1;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;

    initial = GenerateVectorDynamic(3);
    initial << 0, 15, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic initial2 = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_,
                                               jobOrder, processorJobVec);
  };

  double timeLimits = 1;
  DAG_Model dagTasks;
  TaskSet tasks;
  TaskSetInfoDerived tasksInfo;
  std::vector<int> chain1;
  ScheduleOptions scheduleOptions;
  std::vector<uint> processorJobVec;
  SFOrder jobOrder;
  VectorDynamic initial;
};

class DAGScheduleOptimizerTest2 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n2_v2.csv", "orig", 1);
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);

    scheduleOptions.processorNum_ = 1;
    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;
  };

  double timeLimits = 1;
  DAG_Model dagTasks;
  TaskSet tasks;
  TaskSetInfoDerived tasksInfo;
  ScheduleOptions scheduleOptions;
};

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

void print(const std::vector<std::vector<TimeInstance>> &results) {
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

TEST_F(DAGScheduleOptimizerTest2, FindAllJobOrderPermutations) {

  std::vector<TimeInstance> seq0{
      {'s', JobCEC(0, 0)}, {'f', JobCEC(0, 0)}, {'s', JobCEC(1, 0)}, {'f', JobCEC(1, 0)}};
  std::vector<TimeInstance> seq1{
      {'s', JobCEC(0, 0)}, {'s', JobCEC(1, 0)}, {'f', JobCEC(0, 0)}, {'f', JobCEC(1, 0)}};
  std::vector<TimeInstance> seq2{
      {'s', JobCEC(0, 0)}, {'s', JobCEC(1, 0)}, {'f', JobCEC(1, 0)}, {'f', JobCEC(0, 0)}};

  std::vector<TimeInstance> seq3{
      {'s', JobCEC(1, 0)}, {'s', JobCEC(0, 0)}, {'f', JobCEC(0, 0)}, {'f', JobCEC(1, 0)}};
  std::vector<TimeInstance> seq4{
      {'s', JobCEC(1, 0)}, {'s', JobCEC(0, 0)}, {'f', JobCEC(1, 0)}, {'f', JobCEC(0, 0)}};
  std::vector<TimeInstance> seq5{
      {'s', JobCEC(1, 0)}, {'f', JobCEC(1, 0)}, {'s', JobCEC(0, 0)}, {'f', JobCEC(0, 0)}};
  std::vector<std::vector<TimeInstance>> instSeqAll{seq0, seq1, seq2, seq3, seq4, seq5};

  std::vector<std::vector<TimeInstance>> instSeqAllActual = FindAllJobOrderPermutations(dagTasks, tasksInfo);
  print(instSeqAllActual);
  EXPECT_EQ(instSeqAll.size(), instSeqAllActual.size());
  if (instSeqAll.size() == instSeqAllActual.size())
    for (uint i = 0; i < instSeqAll.size(); i++) {
      for (uint j = 0; j < instSeqAll[i].size(); j++)
        EXPECT_TRUE(instSeqAll[i][j] == instSeqAllActual[i][j]);
    }
}

TEST_F(DAGScheduleOptimizerTest1, FindAllJobOrderPermutations) {

  std::vector<std::vector<TimeInstance>> instSeqAllActual = FindAllJobOrderPermutations(dagTasks, tasksInfo);
  //   print(instSeqAllActual);
  EXPECT_EQ(90, instSeqAllActual.size());
}

TEST_F(DAGScheduleOptimizerTest2, FindGlobalOptRTDA) {
  double objActual = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  std::cout << "Global optimal RTDA found is: \n" << objActual << "\n";
  EXPECT_EQ(3 + 3, objActual);
}

TEST_F(DAGScheduleOptimizerTest1, FindGlobalOptRTDA) {
  double objActual = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  std::cout << "Global optimal RTDA found is: \n" << objActual << "\n";
  EXPECT_EQ(4 + 4, objActual);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}