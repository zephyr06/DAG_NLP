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

    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 15, 12;
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

class DAGScheduleOptimizerTest2 : public DAGScheduleOptimizerTest1 {
protected:
  void SetUp() override {
    DAGScheduleOptimizerTest1::SetUp();
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                                        processorJobVec);
  }
};
class DAGScheduleOptimizerTest3 : public DAGScheduleOptimizerTest1 {
protected:
  void SetUp() override {
    DAGScheduleOptimizerTest1::SetUp();
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 5;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                                        processorJobVec);
  }
};
class DAGScheduleOptimizerTest4 : public DAGScheduleOptimizerTest1 {
protected:
  void SetUp() override {
    DAGScheduleOptimizerTest1::SetUp();
    dagTasks.chains_.push_back({0, 1});
  }
};

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDDL) {
  AugmentedJacobian augJaco = GetJacobianDDL(dagTasks, tasksInfo);
  EXPECT_EQ(tasksInfo.length, augJaco.jacobian.sum());
  EXPECT_EQ(1, augJaco.jacobian(0, 0));
  VectorDynamic rhsExpect = GenerateVectorDynamic(4);
  rhsExpect << 9, 19, 18, 17;
  EXPECT_EQ(rhsExpect, augJaco.rhs);
  augJaco.print();
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianActivationTime) {
  AugmentedJacobian augJaco = GetJacobianActivationTime(dagTasks, tasksInfo);
  EXPECT_EQ(-1 * tasksInfo.length, augJaco.jacobian.sum());
  EXPECT_EQ(-1, augJaco.jacobian(0, 0));
  VectorDynamic rhsExpect = GenerateVectorDynamic(4);
  rhsExpect << 0, -10, 0, 0;
  EXPECT_EQ(rhsExpect, augJaco.rhs);
  augJaco.print();
}

TEST_F(DAGScheduleOptimizerTest1, GetExecutionTime) {
  JobCEC job(2, 0);
  EXPECT_EQ(3, GetExecutionTime(job, tasksInfo));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder) {
  AugmentedJacobian augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
  jacobianExpect << 1, -1, 0, 0, 0, 1, 0, -1, 0, 0, -1, 1;
  VectorDynamic rhsExpect = GenerateVectorDynamic(3);
  rhsExpect << -1, -1, -3;

  augJaco.print();

  // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
  EXPECT_EQ(rhsExpect, augJaco.rhs);
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_single_core) {
  std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor =
      SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  EXPECT_EQ(4, jobsOrderedEachProcessor[0].size());
  EXPECT_EQ(0, jobsOrderedEachProcessor[0][0].taskId);
  EXPECT_EQ(0, jobsOrderedEachProcessor[0][1].taskId);
  EXPECT_EQ(2, jobsOrderedEachProcessor[0][2].taskId);
  EXPECT_EQ(1, jobsOrderedEachProcessor[0][3].taskId);
}

TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_multi_core) {
  initial = GenerateVectorDynamic(4);
  initial << 0, 10, 0, 12;
  jobOrder = SFOrder(tasksInfo, initial);
  jobOrder.print();
  VectorDynamic _ =
      SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
  std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor =
      SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  EXPECT_EQ(3, jobsOrderedEachProcessor[0].size());
  EXPECT_EQ(1, jobsOrderedEachProcessor[1].size());
  EXPECT_EQ(1, jobsOrderedEachProcessor[1][0].taskId);
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDBF) {
  AugmentedJacobian augJaco =
      GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
  jacobianExpect << 1, -1, 0, 0, 0, 1, 0, -1, 0, 0, -1, 1;
  VectorDynamic rhsExpect = GenerateVectorDynamic(3);
  rhsExpect << -1, -1, -3;
  augJaco.print();
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder_non_continuous_s_f) {
  initial = GenerateVectorDynamic(4);
  initial << 0, 10, 0, 12;
  jobOrder = SFOrder(tasksInfo, initial);
  jobOrder.print();
  AugmentedJacobian augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(5, 4);
  jacobianExpect << 1, 0, -1, 0, -1, 0, 1, 0, 1, 0, -1, 0, 0, -1, 1, 0, 0, 1, 0, -1;
  VectorDynamic rhsExpect = GenerateVectorDynamic(5);
  rhsExpect << 0, 1, 1, -2, -1;

  augJaco.print();

  // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
  EXPECT_EQ(rhsExpect, augJaco.rhs);
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, StackAugJaco) {
  AugmentedJacobian augJacoDDL = GetJacobianDDL(dagTasks, tasksInfo);
  AugmentedJacobian augJacoAct = GetJacobianActivationTime(dagTasks, tasksInfo);
  AugmentedJacobian augJacoDBF =
      GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  AugmentedJacobian augJacobAll = StackAugJaco(augJacoDDL, augJacoAct);
  augJacobAll = StackAugJaco(augJacobAll, augJacoDBF);
  augJacobAll.print();
  EXPECT_EQ(4 + 4 + 3, augJacobAll.jacobian.rows());
  EXPECT_EQ(4 + 4 + 6, augJacobAll.jacobian.squaredNorm());
  EXPECT_EQ(48, augJacobAll.rhs.sum());
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianAll) {
  AugmentedJacobian augJacobAll =
      GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  augJacobAll.print();

  EXPECT_EQ(4 + 4 + 3 + 3, augJacobAll.jacobian.rows());
  EXPECT_EQ(4 + 4 + 6 + 6, augJacobAll.jacobian.squaredNorm());
  EXPECT_EQ(43, augJacobAll.rhs.sum());
}

TEST_F(DAGScheduleOptimizerTest1, GetVariableBlocksOrdered) {
  std::vector<AugmentedJacobian> augJacos =
      GetVariableBlocksOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  MatrixDynamic jacobianExpect0(4, 4);
  jacobianExpect0 << 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, -1, 0, 0;
  VectorDynamic rhsExpect0(4);
  rhsExpect0 << 9, 0, -1, -1;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect0, augJacos[0].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect0, augJacos[0].rhs));

  MatrixDynamic jacobianExpect1(4, 4);
  jacobianExpect1 << 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, -1, 0;
  VectorDynamic rhsExpect1(4);
  rhsExpect1 << 19, -10, -1, -1;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect1, augJacos[1].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect1, augJacos[1].rhs));

  MatrixDynamic jacobianExpect2(4, 4);
  jacobianExpect2 << 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, -1;
  VectorDynamic rhsExpect2(4);
  rhsExpect2 << 17, 0, -3, -3;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect2, augJacos[2].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect2, augJacos[2].rhs));

  MatrixDynamic jacobianExpect3(2, 4);
  jacobianExpect3 << 0, 0, 0, 1, 0, 0, 0, -1;
  VectorDynamic rhsExpect3(2);
  rhsExpect3 << 18, 0;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect3, augJacos[3].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect3, augJacos[3].rhs));
}

TEST_F(DAGScheduleOptimizerTest2, GetVariableBlock_non_continuous) {

  AugmentedJacobian augJacobAll =
      GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  augJacobAll.print();

  std::vector<AugmentedJacobian> augJacos =
      GetVariableBlocksOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  MatrixDynamic jacobianExpect0(6, 4);
  jacobianExpect0 << 1, 0, 0, 0, -1, 0, 0, 0, 1, 0, -1, 0, 1, -1, 0, 0, -1, 1, 0, 0, 1, -1, 0, 0;
  VectorDynamic rhsExpect0(6);
  rhsExpect0 << 9, 0, -1, 0, 1, 1;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect0, augJacos[0].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect0, augJacos[0].rhs));

  MatrixDynamic jacobianExpect1(3, 4);
  jacobianExpect1 << 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0;
  VectorDynamic rhsExpect1(3);
  rhsExpect1 << 18, 0, -2;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect1, augJacos[1].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect1, augJacos[1].rhs));

  MatrixDynamic jacobianExpect2(4, 4);
  jacobianExpect2 << 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, -1, 0, 0, 1, -1;
  VectorDynamic rhsExpect2(4);
  rhsExpect2 << 19, -10, -1, -1;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect2, augJacos[2].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect2, augJacos[2].rhs));

  MatrixDynamic jacobianExpect3(2, 4);
  jacobianExpect3 << 0, 0, 0, 1, 0, 0, 0, -1;
  VectorDynamic rhsExpect3(2);
  rhsExpect3 << 17, 0;
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect3, augJacos[3].jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect3, augJacos[3].rhs));
}

TEST_F(DAGScheduleOptimizerTest2, MergeAugJacobian) {
  std::vector<AugmentedJacobian> augJacobs =
      GetVariableBlocksOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  AugmentedJacobian jacobAll = MergeAugJacobian(augJacobs);
  EXPECT_EQ(6 + 3 + 4 + 2, jacobAll.jacobian.rows());
  double sum = 0;
  for (uint i = 0; i < augJacobs.size(); i++)
    sum += augJacobs[i].jacobian.sum() + augJacobs[i].rhs.sum();
  EXPECT_FLOAT_EQ(sum, jacobAll.jacobian.sum() + jacobAll.rhs.sum());
}

TEST_F(DAGScheduleOptimizerTest2, MergeAugJacobian_triplets) {
  std::vector<AugmentedJacobianTriplet> augJacobsTriplet =
      GetDAGJacobianTripletOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  AugmentedJacobian jacobAll = MergeAugJacobian(augJacobsTriplet);
  EXPECT_EQ(6 + 3 + 4 + 2, jacobAll.jacobian.rows());

  double sumExpected = 0;
  std::vector<AugmentedJacobian> augJacobs =
      GetVariableBlocksOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  for (uint i = 0; i < augJacobs.size(); i++)
    sumExpected += augJacobs[i].jacobian.sum() + augJacobs[i].rhs.sum();

  EXPECT_FLOAT_EQ(sumExpected, jacobAll.jacobian.sum() + jacobAll.rhs.sum());
}

TEST_F(DAGScheduleOptimizerTest2, overall_reordered_results) {
  AugmentedJacobian augJacobAllOrg =
      GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  augJacobAllOrg.print();

  AugmentedJacobian augJacobAllOrdered =
      GetDAGJacobianOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  augJacobAllOrdered.print();

  EXPECT_EQ(augJacobAllOrg.jacobian.sum(), augJacobAllOrdered.jacobian.sum());
  EXPECT_EQ(augJacobAllOrg.rhs.sum(), augJacobAllOrdered.rhs.sum());
}
TEST_F(DAGScheduleOptimizerTest2, ReOrderLPObj) {
  VectorDynamic c(4);
  c << 1, -1, 0, 0;
  VectorDynamic cExpect(4);
  cExpect << 1, 0, -1, 0;
  VectorDynamic cOrdered = ReOrderLPObj(c, jobOrder, tasksInfo);
  EXPECT_TRUE(gtsam::assert_equal(cExpect, cOrdered));
}
TEST_F(DAGScheduleOptimizerTest2, ReOrderPerformance) {
  AugmentedJacobian jacobAllOrg =
      GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  AugmentedJacobian jacobAllOrdered =
      GetDAGJacobianOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  VectorDynamic c(4);
  c << 1, -1, 0, 0;
  VectorDynamic cOrdered = ReOrderLPObj(c, jobOrder, tasksInfo);

  // std::cout << "Oroginal Jacobian and rhs:\n"
  //           << jacobAllOrg.jacobian << "\n"
  //           << jacobAllOrg.rhs << "\n\n";
  // std::cout << "Ordered Jacobian and rhs:\n"
  //           << jacobAllOrdered.jacobian << "\n"
  //           << jacobAllOrdered.rhs << "\n";
  SpVec cSp(c.sparseView());
  VectorDynamic xActualFromOrg = SolveLP(jacobAllOrg.jacobian.sparseView(), jacobAllOrg.rhs, cSp);
  SpVec cSpO(cOrdered.sparseView());
  VectorDynamic xActualFromOrdered =
      SolveLP(jacobAllOrdered.jacobian.sparseView(), jacobAllOrdered.rhs, cSpO);
  EXPECT_FLOAT_EQ(c.transpose() * xActualFromOrg, cOrdered.transpose() * xActualFromOrdered);
}

TEST_F(DAGScheduleOptimizerTest1, GetJobStartInstancePosition) {
  JobCEC job1(0, 0);
  EXPECT_EQ(0, jobOrder.GetJobStartInstancePosition(job1));
  JobCEC job2(0, 2);
  EXPECT_EQ(8, jobOrder.GetJobStartInstancePosition(job2));
  JobCEC job3(2, 0);
  EXPECT_EQ(4, jobOrder.GetJobStartInstancePosition(job3));
  JobCEC job4(2, 1);
  EXPECT_EQ(4 + 8, jobOrder.GetJobStartInstancePosition(job4));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianCauseEffectChainOrg) {
  AugmentedJacobian augJaco = GetJacobianCauseEffectChainOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                             scheduleOptions.processorNum_, chain1, 0);
  // reason for 5 rows: there are 2 chains within a hyper-period, 2+2 chains in total; the last chain doesn't
  // account for DA;
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(5, 4 + 2);
  jacobianExpect << -1, 0, 0, 1, -1, 0, 0, -1, 0, 1, -1, 0, -1, 0, 0, 1, -1, 0, 0, -1, 0, 1, 0, -1, 0, -1, 0,
      1, -1, 0;
  VectorDynamic rhsExpect = GenerateVectorDynamic(5);
  rhsExpect << -3, -3, -3, -3, -3;

  augJaco.print();

  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest2, GetJacobianCauseEffectChainOrg) {
  AugmentedJacobian augJaco = GetJacobianCauseEffectChainOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                             scheduleOptions.processorNum_, chain1, 1);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(5, 4 + 2 + 2);
  jacobianExpect << -1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 1, 0, 0, -1, 0, -1, 0, 0, 1, 0, 0, -1, 0, 0, -1, 0, 1,
      0, 0, 0, -1, 0, -1, 0, 1, 0, 0, -1, 0;
  VectorDynamic rhsExpect = GenerateVectorDynamic(5);
  rhsExpect << -3, -3, -3, -3, -3;

  augJaco.print();

  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest3, GetJacobianCauseEffectChainOrg) {
  AugmentedJacobian augJaco = GetJacobianCauseEffectChainOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                             scheduleOptions.processorNum_, chain1, 0);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(6, 4 + 2);
  jacobianExpect << -1, 0, 0, 1, -1, 0, // (0,0) -> (2,0) RT
      0, -1, 0, 1, -1, 0,               // (0,1) -> (2,0) RT
      -1, 0, 0, 1, 0, -1,               // (0,0) -> (2,0) DA
      -1, 0, 0, 1, -1, 0,               // (0,0) -> (2,0) RT, next hyper-period
      0, -1, 0, 1, -1, 0,               // (0,1) -> (2,0) RT, next hyper-period
      -1, 0, 0, 1, 0, -1;               // (0,0) -> (2,0) DA, next hyper-period
  VectorDynamic rhsExpect = GenerateVectorDynamic(6);
  rhsExpect << -3, -23, -3, -3, -23, -3;

  augJaco.print();

  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, MergeJacobianOfRTDAChains_single_chain) {
  AugmentedJacobian augJacobRTDA = GetJacobianCauseEffectChainOrg(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, chain1, 0);

  AugmentedJacobian jacobRTDAS = MergeJacobianOfRTDAChains(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                           scheduleOptions.processorNum_);

  EXPECT_TRUE(gtsam::assert_equal(augJacobRTDA.jacobian, jacobRTDAS.jacobian));
}

TEST_F(DAGScheduleOptimizerTest4, MergeJacobianOfRTDAChains_multi_chain) {
  AugmentedJacobian augJacobRTDA0 = GetJacobianCauseEffectChainOrg(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, dagTasks.chains_[0], 0);
  MatrixDynamic temp1(augJacobRTDA0.jacobian.rows(), 2);
  temp1.setZero();
  MatrixDynamic temp2(augJacobRTDA0.jacobian.rows(), augJacobRTDA0.jacobian.cols() + 2);
  temp2 << augJacobRTDA0.jacobian, temp1;

  AugmentedJacobian augJacobRTDA1 = GetJacobianCauseEffectChainOrg(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, dagTasks.chains_[1], 1);

  MatrixDynamic jacobianExpect(augJacobRTDA1.jacobian.rows() + augJacobRTDA0.jacobian.rows(),
                               augJacobRTDA1.jacobian.cols());
  jacobianExpect << temp2, augJacobRTDA1.jacobian;
  VectorDynamic rhsExpect(augJacobRTDA0.jacobian.rows() + augJacobRTDA1.jacobian.rows());
  rhsExpect << augJacobRTDA0.rhs, augJacobRTDA1.rhs;

  AugmentedJacobian jacobRTDAS = MergeJacobianOfRTDAChains(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                                           scheduleOptions.processorNum_);

  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, jacobRTDAS.jacobian));
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, jacobRTDAS.rhs));
}

TEST_F(DAGScheduleOptimizerTest1, GenerateRTDALPOrg) {
  AugmentedJacobian augJacobRTDA = GetJacobianCauseEffectChainOrg(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, chain1, 0);
  augJacobRTDA.print();
  AugmentedJacobian augJacobConstraints =
      GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
  augJacobConstraints.print();
  MatrixDynamic temp1(augJacobConstraints.jacobian.rows(), 2);
  temp1.setZero();
  MatrixDynamic temp2(augJacobConstraints.jacobian.rows(), augJacobRTDA.jacobian.cols());
  temp2 << augJacobConstraints.jacobian, temp1;

  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(
      augJacobRTDA.jacobian.rows() + augJacobConstraints.jacobian.rows(), augJacobRTDA.jacobian.cols());
  jacobianExpect << temp2, augJacobRTDA.jacobian;
  VectorDynamic bExpect =
      GenerateVectorDynamic(augJacobRTDA.jacobian.rows() + augJacobConstraints.jacobian.rows());
  bExpect << augJacobConstraints.rhs, augJacobRTDA.rhs;
  AugmentedJacobian augAll(jacobianExpect, bExpect);
  augAll.print();

  VectorDynamic cExpect = GenerateVectorDynamic(6);
  cExpect << 0, 0, 0, 0, 1, 1;

  LPData lpAll =
      GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect,
                                  lpAll.A_.block(0, 0, jacobianExpect.rows(), jacobianExpect.cols())));
  EXPECT_TRUE(gtsam::assert_equal(bExpect, lpAll.b_));
  EXPECT_TRUE(gtsam::assert_equal(cExpect, lpAll.c_.block(0, 0, cExpect.rows(), cExpect.cols())));
}

class DAGScheduleOptimizerTest5 : public DAGScheduleOptimizerTest1 {
protected:
  void SetUp() override {
    DAGScheduleOptimizerTest1::SetUp();
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 1, 0;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder,
                                        processorJobVec);
  }
};

TEST_F(DAGScheduleOptimizerTest5, jobChainReactionMap) {
  std::unordered_map<JobCEC, std::vector<JobCEC>> reactionChainMap = GetReactionChainMap(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, dagTasks.chains_[0], 0);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 0)][1].taskId);
  EXPECT_EQ(1, reactionChainMap[JobCEC(0, 0)][1].jobId);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 1)][1].taskId);
  EXPECT_EQ(1, reactionChainMap[JobCEC(0, 1)][1].jobId);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 2)][1].taskId);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 2)][1].jobId);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 3)][1].taskId);
  EXPECT_EQ(2, reactionChainMap[JobCEC(0, 3)][1].jobId);
}

TEST_F(DAGScheduleOptimizerTest5, GetJacobianJobOrderReduced) {
  std::unordered_map<JobCEC, std::vector<JobCEC>> reactionChainMap = GetReactionChainMap(
      dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, dagTasks.chains_[0], 0);
  // order: s_03, s_02, s_01, s_00
  AugmentedJacobian augJaco = GetJacobianJobOrderReduced(dagTasks, tasksInfo, jobOrder, 0, reactionChainMap);
  MatrixDynamic jacobianExpect = GenerateMatrixDynamic(8, 4);
  jacobianExpect << 0, 1, 0, -1, // s_01+H + c_0 < s_20 + 2H
      0, -1, 0, 1,               // s_20+H < s_01+H + c_0

      1, 0, 0, -1, // s_00+H + c_0 < s_20 + 2H
      -1, 0, 0, 1, // s_20 + H < s_00+H + c_0,

      0, 1, 0, -1, // s_01 + c_0 < s_20 + H
      0, -1, 0, 1, // s_20 < s_01 + c_0

      1, 0, 0, -1, // s_00 + c_0 < s_20 + H
      -1, 0, 0, 1; // s_20 < s_00 + c_0, this is a first-reaction map

  VectorDynamic rhsExpect = GenerateVectorDynamic(8);
  rhsExpect << 20 - 1 - 1e-5, 1 - 1e-5, //
      20 - 1 - 1e-5, 1 - 1e-5,          //
      20 - 1 - 1e-5, 1 - 1e-5,          //
      20 - 1 - 1e-5, 1 - 1e-5;

  augJaco.print();

  // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
  EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs, 1e-3));
  EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian, 1e-4));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}