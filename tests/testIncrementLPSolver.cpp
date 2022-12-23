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
    // dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig", 1);

    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + testDataSetName + ".csv",
                             "orig", 1);
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
    jobOrder = SFOrder(tasksInfo, initial);
    // jobOrder.print();

    LPData lpDataPrev = GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                          scheduleOptions.processorNum_, true);
    // std::cout << "A_:\n" << MatrixDynamic(lpDataPrev.A_) << std::endl;
    prevOptStv = SolveLP(lpDataPrev);
    RoundIPMResults(prevOptStv);

    // std::cout << "Previous optimal solution: " << prevOptStv << std::endl;
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
  VectorDynamic prevOptStv;
  LPData lpDataPrev;
};

CentralVariable LPData::GenerateInitialLPWarmStart(const VectorDynamic &warmStartX,
                                                   const DAG_Model &dagTasks) {
  //   VectorDynamic xCurr_ = A_.transpose() * (AASolver_.solve(b_));
  TaskSetInfoDerived tasksInfo = TaskSetInfoDerived(dagTasks.tasks);
  int m = A_.rows();
  int artVarNum = dagTasks.chains_.size() * 2;
  int n = warmStartX.rows() - artVarNum;

  VectorDynamic xCurr_(m + n + artVarNum);
  xCurr_.block(0, 0, warmStartX.rows(), 1) = warmStartX;

  for (uint i = 0; i < dagTasks.chains_.size(); i++) {
    RTDA rtdaMax = GetMaxRTDA(tasksInfo, dagTasks.chains_[i], warmStartX);
    xCurr_(warmStartX.rows() + i * 2) = rtdaMax.reactionTime;
    xCurr_(warmStartX.rows() + i * 2 + 1) = rtdaMax.dataAge;
  }
  MatrixDynamic A = A_.block(0, 0, A_.rows(), warmStartX.rows());
  xCurr_.block(n + artVarNum, 0, m, 1) = b_ - A * warmStartX;

  VectorAdd(xCurr_, std::max(-1.5 * xCurr_.minCoeff(), 0.0));

  VectorDynamic lambdaCurr_ = AASolver_.solve(A_ * c_);
  if ((eigen_is_nan(lambdaCurr_)))
    CoutError("centralDelta.lambda becomes Nan during iterations!");
  VectorDynamic sCurr_ = c_ - A_.transpose() * lambdaCurr_;
  VectorAdd(sCurr_, std::max(-1.5 * sCurr_.minCoeff(), 0.0));
  double deltax = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / sCurr_.sum() / 2.0;
  double deltas = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / xCurr_.sum() / 2.0;
  VectorAdd(xCurr_, deltax);
  VectorAdd(sCurr_, deltas);
  EndTimer("GenerateInitialIPM");
  return CentralVariable{xCurr_, sCurr_, lambdaCurr_};
}

TEST_F(LPTest1, basic) {
  SFOrder jobOrderPermutation = jobOrder;
  JobCEC jobCurr(1, 0);
  jobOrderPermutation.RemoveJob(jobCurr);
  jobOrderPermutation.InsertStart(jobCurr, 4);
  jobOrderPermutation.InsertFinish(jobCurr, 5);
  //   jobOrderPermutation.print();

  VectorDynamic expectX =
      OptRTDA_IPMOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_, true);

  LPData lpData = GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrderPermutation, processorJobVec,
                                    scheduleOptions.processorNum_, true);
  lpData.centralVarCurr_ = lpData.GenerateInitialLPWarmStart(expectX, dagTasks);
  VectorDynamic startTimeVectorAfterOpt = SolveLP(lpData);
  RoundIPMResults(startTimeVectorAfterOpt);
  //   EXPECT_TRUE(gtsam::assert_equal(expectX, startTimeVectorAfterOpt));

  std::cout << Color::blue << "Obj from Old LPOrderScheduler: "
            << RTDAExperimentObj::TrueObj(
                   dagTasks, tasksInfo, expectX.block(0, 0, expectX.rows(), initial.cols()), scheduleOptions)
            << Color::def << std::endl;
  std::cout << Color::blue << "Obj after incremental LPOrderScheduler: "
            << RTDAExperimentObj::TrueObj(dagTasks, tasksInfo,
                                          startTimeVectorAfterOpt.block(0, 0, initial.rows(), initial.cols()),
                                          scheduleOptions)
            << Color::def << std::endl;

  std::cout << "Old X range: " << expectX.minCoeff() << ", " << expectX.maxCoeff() << std::endl;
  std::cout << "New X range: " << startTimeVectorAfterOpt.minCoeff() << ", "
            << startTimeVectorAfterOpt.maxCoeff() << std::endl;
  std::cout << "Old obj from LP: " << lpData.c_.transpose() * expectX << std::endl;
  std::cout << "Incremental obj from LP: " << lpData.c_.transpose() * startTimeVectorAfterOpt << std::endl;

  // std::cout << "A_\n" << MatrixDynamic(lpData.A_) << std::endl;
  // WriteMatrixToFile("A.txt", MatrixDynamic(lpData.A_));

  std::cout << " A range: " << (MatrixDynamic(lpData.A_).array()).minCoeff() << ", "
            << (MatrixDynamic(lpData.A_).array()).maxCoeff() << std::endl;
  std::cout << "A size: " << lpData.A_.rows() << ", " << lpData.A_.cols() << std::endl;
  std::cout << "Constraint violation Ax incremental  LP: "
            << (MatrixDynamic(lpData.A_) * lpData.centralVarCurr_.x).array().maxCoeff() << std::endl;
  VectorDynamic Ab = lpData.A_ * expectX;

  MatrixDynamic A = MatrixDynamic(lpData.A_);
  for (uint i = 0; i < Ab.rows(); i++) {
    if (Ab(i) > 1e202) {
      for (uint j = 0; j < A.cols(); j++) {
        if (A(i, j) != 0) {
          std::cout << "(" << i << ", " << j << "): " << A(i, j) << " ,";
          std::cout << expectX(j) << std::endl;
          std::cout << expectX.array().maxCoeff() << std::endl;
        }
      }
      std::cout << "\n\n\n";
    }
  }
  WriteMatrixToFile("X.txt", expectX);

  std::cout << "Constraint violation about direct LP: " << (lpData.b_ - lpData.A_ * expectX).maxCoeff()
            << std::endl;
  std::cout << "Constraint violation about Incremental LP: "
            << (lpData.b_ - lpData.A_ * lpData.centralVarCurr_.x).maxCoeff() << std::endl;
  std::cout << "Nan: " << eigen_is_nan(expectX) << ", " << eigen_is_nan(startTimeVectorAfterOpt) << std::endl;
  std::cout << "Inf: " << eigen_is_inf(expectX) << ", " << eigen_is_inf(startTimeVectorAfterOpt) << ", "
            << eigen_is_inf(lpData.b_) << ", " << eigen_is_inf((lpData.b_ - lpData.A_ * expectX))
            << std::endl;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}