#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

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

TEST_F(DAGScheduleOptimizerTest1, OptimizeWithJobOrder)
{
    ScheduleOptimizer optimizer(dagTasks);
    optimizer.OptimizeWithJobOrder(initial, processorJobVec, jobOrder);
    VectorDynamic stvRes = optimizer.getOptimizedStartTimeVector();
    VectorDynamic expected = initial;
    expected << 9, 10, 18, 11;
    EXPECT_EQ(expected, stvRes);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}