#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "gtsam/base/Testable.h" // assert_equal

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/JacobianAnalyze.h"

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

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDDL)
{
    auto augJaco = GetJacobianDDL(dagTasks, tasksInfo);
    EXPECT_EQ(tasksInfo.length, augJaco.jacobian.sum());
    EXPECT_EQ(1, augJaco.jacobian(0, 0));
    VectorDynamic rhsExpect = GenerateVectorDynamic(4);
    rhsExpect << 10, 20, 20, 20;
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    augJaco.print();
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianActivationTime)
{
    auto augJaco = GetJacobianActivationTime(dagTasks, tasksInfo);
    EXPECT_EQ(-1 * tasksInfo.length, augJaco.jacobian.sum());
    EXPECT_EQ(-1, augJaco.jacobian(0, 0));
    VectorDynamic rhsExpect = GenerateVectorDynamic(4);
    rhsExpect << 0, -10, 0, 0;
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    augJaco.print();
}

TEST_F(DAGScheduleOptimizerTest1, GetExecutionTime)
{
    JobCEC job(2, 0);
    EXPECT_EQ(3, GetExecutionTime(job, tasksInfo));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder)
{
    auto augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
    jacobianExpect << 1, -1, 0, 0,
        0, 1, 0, -1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(3);
    rhsExpect << -1, -1, -3;

    augJaco.print();

    // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_single_core)
{
    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    EXPECT_EQ(4, jobsOrderedEachProcessor[0].size());
    EXPECT_EQ(0, jobsOrderedEachProcessor[0][0].taskId);
    EXPECT_EQ(0, jobsOrderedEachProcessor[0][1].taskId);
    EXPECT_EQ(2, jobsOrderedEachProcessor[0][2].taskId);
    EXPECT_EQ(1, jobsOrderedEachProcessor[0][3].taskId);
}

TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_multi_core)
{
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    EXPECT_EQ(3, jobsOrderedEachProcessor[0].size());
    EXPECT_EQ(1, jobsOrderedEachProcessor[1].size());
    EXPECT_EQ(1, jobsOrderedEachProcessor[1][0].taskId);
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDBF)
{
    auto augJaco = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
    jacobianExpect << 1, -1, 0, 0,
        0, 1, 0, -1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(3);
    rhsExpect << -1, -1, -3;
    augJaco.print();
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder_non_continuous_s_f)
{
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    auto augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(5, 4);
    jacobianExpect << 1, 0, -1, 0,
        -1, 0, 1, 0,
        1, 0, -1, 0,
        0, -1, 1, 0,
        0, 1, 0, -1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(5);
    rhsExpect << 0, 1, 1, -2, -1;

    augJaco.print();

    // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, StackAugJaco)
{
    AugmentedJacobian augJacoDDL = GetJacobianDDL(dagTasks, tasksInfo);
    AugmentedJacobian augJacoAct = GetJacobianActivationTime(dagTasks, tasksInfo);
    AugmentedJacobian augJacoDBF = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

    AugmentedJacobian augJacobAll = StackAugJaco(augJacoDDL, augJacoAct);
    augJacobAll = StackAugJaco(augJacobAll, augJacoDBF);
    augJacobAll.print();
    EXPECT_EQ(4 + 4 + 3, augJacobAll.jacobian.rows());
    EXPECT_EQ(4 + 4 + 6, augJacobAll.jacobian.squaredNorm());
    EXPECT_EQ(20 + 20 + 20 - 1 - 1 - 3, augJacobAll.rhs.sum());
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianAll)
{
    AugmentedJacobian augJacobAll = GetJacobianAll(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    augJacobAll.print();

    EXPECT_EQ(4 + 4 + 3 + 3, augJacobAll.jacobian.rows());
    EXPECT_EQ(4 + 4 + 6 + 6, augJacobAll.jacobian.squaredNorm());
    EXPECT_EQ(20 + 20 + 20 - 1 - 1 - 3 - 1 - 1 - 3, augJacobAll.rhs.sum());
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}