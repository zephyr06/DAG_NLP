
#include <gtest/gtest.h>
#include "sources/Optimization/OptimizeSFOrder.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;

class ObjExperimentObjTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig"); // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 1, 2};
        dagTasks.chains_[0] = chain1;

        scheduleOptions.considerSensorFusion_ = 1;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
};

TEST_F(ObjExperimentObjTest1, constructor)
{
    EXPECT_EQ(tasks.size(), 5);
}

// TODO: write a failure test for OptimizeOrder

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}