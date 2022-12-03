#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"

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

struct AugmentedJacobian
{
    MatrixDynamic jacobian;
    VectorDynamic rhs;
    void print()
    {
        std::cout << "Jacobian: " << jacobian << std::endl;
        std::cout << "RHS: " << rhs << std::endl;
    }
};
AugmentedJacobian GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
{
    MatrixDynamic jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);

    VectorDynamic rhs = GenerateVectorDynamic(tasksInfo.length);
    int count = 0;
    for (uint i = 0; i < dagTasks.tasks.size(); i++)
    {
        for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
        {
            jacobian(count, count) = 1;
            rhs(count++, 0) = GetDeadline(JobCEC{(int)i, j}, tasksInfo);
        }
    }
    return AugmentedJacobian{jacobian, rhs};
}

AugmentedJacobian GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
{
    MatrixDynamic jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);

    VectorDynamic rhs = GenerateVectorDynamic(tasksInfo.length);
    int count = 0;
    for (uint i = 0; i < dagTasks.tasks.size(); i++)
    {
        for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
        {
            jacobian(count, count) = -1;
            rhs(count++, 0) = GetActivationTime(JobCEC{(int)i, j}, tasksInfo) * -1;
        }
    }
    return AugmentedJacobian{jacobian, rhs};
}

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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}