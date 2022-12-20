#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "sources/Factors/JacobianAnalyze.h"
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
class LPTest1 : public ::testing::Test {
protected:
  void SetUp() override {
    A = MatrixDynamic(15, 4);
    A << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1,
        1, 0, -1, 0, 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 1, 0, -1, 0, 0, 0, 1, -1, 0, -1, 0, 1;
    As = A.sparseView();
    b = VectorDynamic(15);
    b << 10, 20, 20, 20, -0, -10, 0, -0, -1, -2, 0, 1, -1, 1, -3;
    c = VectorDynamic(4);
    c << 0, 1, 0, 1;
    // VectorDynamic xExpect(4);
    // xExpect << 0, 10, 1, 0;
    // for (int k = 0; k < As.outerSize(); ++k)
    // {
    //     for (Eigen::SparseMatrix<double>::InnerIterator it(As, k); it; ++it)
    //     {
    //         std::cout << "(" << it.row() << ","; // row index
    //         std::cout << it.col() << ")\t";      // col index (here it is equal to k)
    //     }
    // }
  };

  MatrixDynamic A;
  Eigen::SparseMatrix<double> As;
  VectorDynamic b;
  VectorDynamic c;
};

TEST_F(LPTest1, LPData_constructor) {
  LPData lpData(As, b, c.sparseView());
  EXPECT_EQ(A.sum() + 15, lpData.A_.sum());
  EXPECT_EQ(19, lpData.A_.cols());
  EXPECT_EQ(c.sum() + 0, lpData.c_.sum());
  EXPECT_TRUE(gtsam::assert_equal(b, lpData.b_));
}

TEST_F(LPTest1, GenerateInitialLP) {
  LPData lpData(As, b, c.sparseView());
  VectorDynamic x0Expect = GenerateVectorDynamic(19);
  x0Expect << 6.5639, 11.4870, 8.5062, 8.0062, 7.3844, 12.4613, 15.4421, 15.9421, 6.5639, 1.4870, 8.5062,
      8.0062, 2.9165, 2.9549, 3.4165, 1.5318, 2.9165, 2.4742, 2.4549;
  VectorDynamic s0Expect = GenerateVectorDynamic(19);
  s0Expect << 0.5843, 0.7574, 0.5891, 0.7141, 0.4048, 0.2317, 0.4000, 0.2750, 0.5843, 0.7574, 0.5891, 0.7141,
      0.4994, 0.6628, 0.6244, 0.3647, 0.4994, 0.6195, 0.5378;
  VectorDynamic lambda0Expect = GenerateVectorDynamic(15);
  lambda0Expect << 0.0897, 0.2628, 0.0946, 0.2196, -0.0897, -0.2628, -0.0946, -0.2196, -0.0048, -0.1683,
      -0.1298, 0.1298, -0.0048, -0.1250, -0.0433;

  EXPECT_TRUE(gtsam::assert_equal(x0Expect, lpData.centralVarCurr_.x, 1e-4));
  EXPECT_TRUE(gtsam::assert_equal(s0Expect, lpData.centralVarCurr_.s, 1e-4));
  EXPECT_TRUE(gtsam::assert_equal(lambda0Expect, lpData.centralVarCurr_.lambda, 1e-4));
}

TEST_F(LPTest1, SolveLP) {

  TimerFunc _;
  VectorDynamic xExpect(4);
  xExpect << 0, 10, 1, 0;
  VectorDynamic xActual = SolveLP(As, b, c.sparseView());
  EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual, 1e-3));
}

class LPTest2 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v11.csv", "orig", 1);
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
    // chain1 = {0, 2};
    // dagTasks.chains_[0] = chain1;
    timeLimits = 10;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
    // std::cout << "Initial solutions: " << std::endl
    //           << initial << std::endl
    //           << std::endl;
    jobOrder = SFOrder(tasksInfo, initial);
    // jobOrder.print();

    // std::vector<AugmentedJacobian> augJacobs = GetVariableBlocksOrdered(dagTasks, tasksInfo, jobOrder,
    // processorJobVec, scheduleOptions.processorNum_); AugmentedJacobian jacobAll =
    // MergeAugJacobian(augJacobs);
    AugmentedJacobian jacobAll =
        GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    A = jacobAll.jacobian;
    As = A.sparseView();

    // std::cout << "A" << std::endl
    //           << A << std::endl
    //           << std::endl;
    b = jacobAll.rhs;
    // std::cout << "rhs" << std::endl
    //           << b << std::endl
    //           << std::endl;
    c = GenerateVectorDynamic(A.cols());
    c(0) = 1;
    c(1) = -1;
    // std::cout << c << std::endl
    //           << std::endl;
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

  MatrixDynamic A;
  Eigen::SparseMatrix<double> As;
  VectorDynamic b;
  VectorDynamic c;
};

TEST_F(LPTest2, formulation_feasible) { ASSERT_THAT((b - A * initial).minCoeff(), testing::Ge(0)); }

TEST_F(LPTest2, SolveLP_Cplex) {
  TimerFunc _;
  VectorDynamic xExpect(2);
  xExpect << 0, 19;
  VectorDynamic xActual = SolveLP_Cplex(As, b, c);
  EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual.block(0, 0, 2, 1), 1e-4));
}

TEST_F(LPTest2, SolveLP) {
  TimerFunc _;
  VectorDynamic xExpect(2);
  xExpect << 0, 19;
  VectorDynamic xActual = SolveLP(As, b, c.sparseView());
  EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual.block(0, 0, 2, 1), 1e-3));
}
class LPTest3 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v22.csv", "orig", 1);
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
    // chain1 = {0, 2};
    // dagTasks.chains_[0] = chain1;
    timeLimits = 10;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
    // std::cout << "Initial solutions: " << std::endl
    //           << initial << std::endl
    //           << std::endl;
    jobOrder = SFOrder(tasksInfo, initial);
    if (GlobalVariablesDAGOpt::PrintOutput)
      jobOrder.print();
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

  MatrixDynamic A;
  Eigen::SparseMatrix<double> As;
  VectorDynamic b;
  VectorDynamic c;
};

TEST_F(LPTest3, SolveLP_lessJobOrderConstraints) {
  TimerFunc _;
  std::cout << Color::blue << "Initial solution: "
            << RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initial, scheduleOptions) << Color::def
            << std::endl;
  if (GlobalVariablesDAGOpt::PrintOutput) {
    std::cout << "Initial solution:\n" << initial << std::endl;
  }
  BeginTimer("main");
  VectorDynamic xActual;
  if (GlobalVariablesDAGOpt::ReOrderProblem == "orig")
    xActual =
        OptRTDA_IPMOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, true);
  else if (GlobalVariablesDAGOpt::ReOrderProblem == "band") {
    // jacobAll =
    //     GetDAGJacobianOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec,
    //     scheduleOptions.processorNum_);
    // c = ReOrderLPObj(c, jobOrder, tasksInfo);
  } else if (GlobalVariablesDAGOpt::ReOrderProblem == "amd")
    CoutError("Please provide reorder method implementation!");
  else
    CoutError("Please provide reorder method implementation!");
  EndTimer("main");
  std::cout << "The number of variables is: " << xActual.rows() << std::endl;
  if (GlobalVariablesDAGOpt::PrintOutput)
    std::cout << "optimal start time vector found: " << xActual << std::endl;

  std::cout << Color::blue << "Optimal solution found: "
            << RTDAExperimentObj::TrueObj(
                   dagTasks, tasksInfo, xActual.block(0, 0, initial.rows(), initial.cols()), scheduleOptions)
            << Color::def << std::endl;

  EXPECT_THAT(RTDAExperimentObj::TrueObj(
                  dagTasks, tasksInfo, xActual.block(0, 0, initial.rows(), initial.cols()), scheduleOptions),
              testing::Le(RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initial, scheduleOptions)));
  EXPECT_FALSE(eigen_is_nan(xActual));
  PrintTimer();
}
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}