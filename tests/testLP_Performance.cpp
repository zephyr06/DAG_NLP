#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Jacobi>
#include "gtsam/base/Testable.h" // assert_equal

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/JacobianAnalyze.h"
#include "sources/Utils/IncrementQR.h"
#include "sources/Optimization/LinearProgrammingSolver.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
using namespace LPOptimizer;
class LPTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
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

        VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
        // std::cout << "Initial solutions: " << std::endl
        //           << initial << std::endl
        //           << std::endl;
        jobOrder = SFOrder(tasksInfo, initial);
        // jobOrder.print();

        std::vector<AugmentedJacobian> augJacobs = GetVariableBlocks(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
        AugmentedJacobian jacobAll = MergeAugJacobian(augJacobs);
        A = jacobAll.jacobian;

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
    VectorDynamic b;
    VectorDynamic c;
};

TEST_F(LPTest1, SolveLP)
{
    TimerFunc _;
    VectorDynamic xExpect(2);
    xExpect << 0, 15;
    VectorDynamic xActual = SolveLP(A, b, c);
    EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual.block(0, 0, 2, 1), 1e-4));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}