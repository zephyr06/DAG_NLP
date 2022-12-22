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
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig", 1);
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
    jobOrder.print();

    LPData lpDataPrev = GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrder, processorJobVec,
                                          scheduleOptions.processorNum_, true);

    prevOptStv = SolveLP(lpDataPrev);
    RoundIPMResults(prevOptStv);

    std::cout << "Previous optimal solution: " << prevOptStv << std::endl;
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

TEST_F(LPTest1, basic) {
  SFOrder jobOrderPermutation = jobOrder;
  JobCEC jobCurr(1, 0);
  jobOrderPermutation.RemoveJob(jobCurr);
  jobOrderPermutation.InsertStart(jobCurr, 10);
  jobOrderPermutation.InsertFinish(jobCurr, 12);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}