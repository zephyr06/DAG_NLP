#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

// #include "sources/Factors/JacobianAnalyze.h"
#include "sources/Optimization/GlobalOptimization.h"
// #include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Utils/IncrementQR.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
// using namespace LPOptimizer;
// TODO: Fix DDL issue, rhs should minus execution time;
class DAGScheduleOptimizerTest1 : public ::testing::Test
{
protected:
  void SetUp() override
  {
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

class DAGScheduleOptimizerTest2 : public ::testing::Test
{
protected:
  void SetUp() override
  {
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

TEST_F(DAGScheduleOptimizerTest2, IterateTimeInstanceSeq)
{
  PermutationStatus permutationStatus(dagTasks, tasksInfo, scheduleOptions);
  std::vector<TimeInstance> prevSeq;
  prevSeq.reserve(2 * tasksInfo.length);

  std::vector<JobQueueOfATask> jobQueueTaskSet;
  int N = tasksInfo.N;
  jobQueueTaskSet.reserve(N);
  for (int i = 0; i < N; i++)
    jobQueueTaskSet.push_back(
        JobQueueOfATask(dagTasks.tasks[i], tasksInfo.hyperPeriod / dagTasks.tasks[i].period));

  IterateTimeInstanceSeq(prevSeq, jobQueueTaskSet, permutationStatus);
  // EXPECT_EQ(6, permutationStatus.totalPermutation_);
  EXPECT_THAT(6, testing::Ge(permutationStatus.totalPermutation_));
  EXPECT_EQ(6, permutationStatus.valOpt_);

  std::vector<TimeInstance> instanceOrderOpt{
      {'s', JobCEC(0, 0)}, {'f', JobCEC(0, 0)}, {'s', JobCEC(1, 0)}, {'f', JobCEC(1, 0)}};

  for (uint j = 0; j < instanceOrderOpt.size(); j++)
    EXPECT_TRUE(instanceOrderOpt[j] == permutationStatus.orderOpt_.instanceOrder_[j]);
}

TEST_F(DAGScheduleOptimizerTest1, FindAllJobOrderPermutations)
{
  PermutationStatus permSta = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  // std::vector<std::vector<TimeInstance>> instSeqAllActual = FindAllJobOrderPermutations(dagTasks,
  // tasksInfo);
  //   PrintTimeSequence2D(instSeqAllActual);
  // EXPECT_EQ(90, permSta.totalPermutation_);
  EXPECT_THAT(90, testing::Ge(permSta.totalPermutation_));
}

TEST_F(DAGScheduleOptimizerTest2, FindGlobalOptRTDA)
{
  PermutationStatus permSta = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  EXPECT_EQ(3 + 3, permSta.valOpt_);
}

TEST_F(DAGScheduleOptimizerTest1, FindGlobalOptRTDA)
{
  PermutationStatus permSta = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  EXPECT_EQ(4 + 4, permSta.valOpt_);
}

class DAGScheduleOptimizerTest3 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig", 1);
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

TEST_F(DAGScheduleOptimizerTest3, FindGlobalOptRTDA)
{
  PermutationStatus permSta = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  EXPECT_EQ(5 + 4, permSta.valOpt_);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}