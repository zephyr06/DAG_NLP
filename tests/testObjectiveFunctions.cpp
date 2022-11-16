#include <gtest/gtest.h>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/ObjectiveFunctions.h"
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
TEST_F(ObjExperimentObjTest1, RTDAEvaluate1)
{
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5; // the chain starts at 0 and ends at 3+12+200+200=415

    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initialEstimate, scheduleOptions), (414 + 414) * 2 * 0.5 + 414 + 414); // There are two job-level chains even though hyper-period equals 200
}
TEST_F(ObjExperimentObjTest1, SFEvaluate1)
{
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    EXPECT_EQ(RTSS21ICObj::EvaluateRTDA(dagTasks, tasksInfo, initialEstimate, scheduleOptions), (414 + 414) * 2 * 0.5 + (414 + 414) * 10); // There are two job-level chains even though hyper-period equals 200
    EXPECT_EQ(RTSS21ICObj::EvaluateSF(dagTasks, tasksInfo, initialEstimate, scheduleOptions), 4 * 0.5 + 4 * 10);
    EXPECT_EQ(RTSS21ICObj::Evaluate(dagTasks, tasksInfo, initialEstimate, scheduleOptions), 42 + 9108);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}