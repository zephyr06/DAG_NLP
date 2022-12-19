#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "sources/Optimization/JacobianAnalyze.h"
#include "sources/Optimization/LPSolverCplex.h"
#include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/IncrementQR.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
using namespace LPOptimizer;

class DAGScheduleOptimizerTest1 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/"
                                                                   "test_n3_v18.csv",
                             "orig", 1);
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

    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 15, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    xOpt = GenerateVectorDynamic(4);
    xOpt << 9, 10, 18, 11;
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
  VectorDynamic xOpt;
};

class DAGScheduleOptimizerTest2 : public DAGScheduleOptimizerTest1 {
protected:
  void SetUp() override {
    DAGScheduleOptimizerTest1::SetUp();
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
    // std::cout << "Initial solutions: " << std::endl
    //           << initial << std::endl
    //           << std::endl;
    jobOrder = SFOrder(tasksInfo, initial);
    if (GlobalVariablesDAGOpt::PrintOutput)
      jobOrder.print();
  }
};

TEST_F(DAGScheduleOptimizerTest1, OptRTDA_IPM) {
  VectorDynamic startOpt =
      LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
  std::cout << startOpt << std::endl;
  EXPECT_TRUE(gtsam::assert_equal(xOpt, startOpt));
  VectorDynamic startFromIPM =
      OptRTDA_IPMOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  EXPECT_EQ(RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startOpt, scheduleOptions),
            RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startFromIPM.block(0, 0, 4, 1), scheduleOptions));
}

// TODO: add expect assertion
TEST_F(DAGScheduleOptimizerTest2, SolveLP) {
  // VectorDynamic xActual =
  //     OptRTDA_IPMOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  std::cout << "Initial solution: " << initial << std::endl;
  LPData lpData =
      GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  lpData.print();
  VectorDynamic xActual = SolveLP(lpData);
  RoundIPMResults(xActual);

  std::cout << "The number of variables is: " << xActual.rows() << std::endl;
  if (GlobalVariablesDAGOpt::PrintOutput)
    std::cout << "optimal start time vector found: " << xActual << std::endl;
  std::cout << "Optimal solution found: "
            << RTDAExperimentObj::TrueObj(
                   dagTasks, tasksInfo, xActual.block(0, 0, initial.rows(), initial.cols()), scheduleOptions)
            << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}