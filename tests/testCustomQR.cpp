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

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class DAGScheduleOptimizerTest1 : public ::testing::Test
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

        initial = GenerateVectorDynamic(4);
        initial << 0, 10, 15, 12;
        jobOrder = SFOrder(tasksInfo, initial);
        jobOrder.print();
        VectorDynamic initial2 = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
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

TEST_F(DAGScheduleOptimizerTest1, QR_Eigen)
{
    AugmentedJacobian augJacobAll = GetJacobianAll(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

    QR qrEigen = GetEigenQR(augJacobAll.jacobian);
    qrEigen.print();
    EXPECT_FLOAT_EQ(-5.39103, qrEigen.R.sum());
}

// Apply given rotations at row-basis; After applying it, the m(i, 0) will become 0
inline Eigen::JacobiRotation<float> ApplyGivenRotations(MatrixDynamic &m, int rowIndex)
{
    Eigen::JacobiRotation<float> G;
    G.makeGivens(m(rowIndex - 1, 0), m(rowIndex, 0));
    m.applyOnTheLeft(rowIndex - 1, rowIndex, G.adjoint());
    return G;
}
TEST_F(DAGScheduleOptimizerTest1, makeGivens)
{
    MatrixDynamic m = GenerateMatrixDynamic(4, 3);
    m << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
    auto g1 = ApplyGivenRotations(m, 3);
    EXPECT_NEAR(0, m(3, 0), 1e-5);
    auto g2 = ApplyGivenRotations(m, 2);
    EXPECT_NEAR(0, m(2, 0), 1e-5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}