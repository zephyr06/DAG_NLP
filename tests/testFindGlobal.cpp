#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "sources/Factors/JacobianAnalyze.h"
#include "sources/Optimization/GlobalOptimization.h"
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
  PrintTimeSequence2D(instSeqAllActual);
  EXPECT_EQ(instSeqAll.size(), instSeqAllActual.size());
  if (instSeqAll.size() == instSeqAllActual.size())
    for (uint i = 0; i < instSeqAll.size(); i++) {
      for (uint j = 0; j < instSeqAll[i].size(); j++)
        EXPECT_TRUE(instSeqAll[i][j] == instSeqAllActual[i][j]);
    }
}

TEST_F(DAGScheduleOptimizerTest1, FindAllJobOrderPermutations) {

  std::vector<std::vector<TimeInstance>> instSeqAllActual = FindAllJobOrderPermutations(dagTasks, tasksInfo);
  //   PrintTimeSequence2D(instSeqAllActual);
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

class DAGScheduleOptimizerTest3 : public ::testing::Test {
protected:
  void SetUp() override {
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

TEST_F(DAGScheduleOptimizerTest3, FindGlobalOptRTDA) {
  double objActual = FindGlobalOptRTDA(dagTasks, tasksInfo, scheduleOptions);
  std::cout << "Global optimal RTDA found is: \n" << objActual << "\n";
  EXPECT_EQ(5 + 4, objActual);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}