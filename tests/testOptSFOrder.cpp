#include <gtest/gtest.h>
#include "sources/Optimization/OptimizeSFOrder.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class ScheduleDAGModelTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig"); // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 2};
        dagTasks.chains_[0] = chain1;

        startTimeVector = GenerateVectorDynamic(4);
        startTimeVector << 0, 10, 1, 1;
        sfOrder = SFOrder(tasksInfo, startTimeVector);
        sfOrder.print();

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
    VectorDynamic startTimeVector;
    SFOrder sfOrder;
};
TEST_F(ScheduleDAGModelTest1, FindJobActivateRange)
{
    JobCEC jobRelocate(0, 0);
    JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
    EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
    EXPECT_EQ(7, jobStartFinishInstActiveRange.maxIndex);
}

TEST_F(ScheduleDAGModelTest1, WhetherStartFinishTooLong)
{
    startTimeVector << 0, 10, 1, 3;
    sfOrder = SFOrder(tasksInfo, startTimeVector);
    sfOrder.print();
    JobCEC jobRelocate(0, 0);
    sfOrder.RemoveJob(jobRelocate);
    LLint startP = 0;
    sfOrder.InsertStart(jobRelocate, startP);
    double accumLength = 0;

    LLint finishP = 1;
    EXPECT_FALSE(WhetherStartFinishTooLong(accumLength, jobRelocate, finishP, tasksInfo, sfOrder, startP));

    finishP = 2;
    EXPECT_FALSE(WhetherStartFinishTooLong(accumLength, jobRelocate, finishP, tasksInfo, sfOrder, startP));

    finishP = 3;
    EXPECT_TRUE(WhetherStartFinishTooLong(accumLength, jobRelocate, finishP, tasksInfo, sfOrder, startP));

    finishP = 4;
    EXPECT_TRUE(WhetherStartFinishTooLong(accumLength, jobRelocate, finishP, tasksInfo, sfOrder, startP));

    finishP = 7;
    EXPECT_TRUE(WhetherStartFinishTooLong(accumLength, jobRelocate, finishP, tasksInfo, sfOrder, startP));
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}