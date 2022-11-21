
#include <gtest/gtest.h>
#include "sources/Optimization/OptimizeSFOrder.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;

class DAGScheduleOptimizerTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig"); // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 2};
        dagTasks.chains_[0] = chain1;

        double timeLimits = 1;
        scheduleOptions.processorNum_ = 2;
        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;

        dagScheduleOptimizer = DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj>(dagTasks, scheduleOptions, timeLimits);
    };

    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
    DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj> dagScheduleOptimizer;
};

TEST_F(DAGScheduleOptimizerTest1, constructor)
{
    EXPECT_EQ(tasks.size(), 3);
}
// TODO: fix the test below
TEST_F(DAGScheduleOptimizerTest1, UpdateStatus)
{
    VectorDynamic initial = GenerateVectorDynamic(4);
    initial << 5, 10, 6, 7;
    SFOrder jobOrder(tasksInfo, initial);
    JobGroupRange uselessRange(0, 100);
    LLint uselessFinishP = 100;
    EXPECT_EQ(2, dagScheduleOptimizer.UpdateStatus(jobOrder, uselessRange, uselessFinishP));
}

// TODO: write a failure test for OptimizeOrder

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}