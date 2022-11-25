#include <gtest/gtest.h>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/ObjectiveFunctions.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/OrderScheduler.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;
class ObjExperimentObjTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig"); // single-rate dag
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

TEST(aaa, nonWorkConserveCase)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    ScheduleOptions scheduleOptions;
    scheduleOptions.causeEffectChainNumber_ = 1;
    scheduleOptions.processorNum_ = 2;

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 17, 18, 1;
    SFOrder sfOrder(tasksInfo, initial);

    VectorDynamic initialSTV = SFOrderScheduling(dagTasks.tasks, tasksInfo, processorNum, sfOrder);
    PrintSchedule(tasksInfo, initialSTV);
    VectorDynamic expect = GenerateVectorDynamic(4);
    expect << 0, 10, 11, 1;
    EXPECT_TRUE(assert_equal(expect, initialSTV));
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initialSTV, scheduleOptions), (4 + 14) + (4 + 4 + 14 - 1 + 4 + 4) * 0.5);

    initial << 2, 10, 0, 0;
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder2(tasksInfo, initial);
    sfOrder2.print();
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial, scheduleOptions), (21 + 13) + (21 + -1 + 13 + 13 + 21 + -1) * 0.5);
}
TEST(Obj, RTDA_v1)
{
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 10, 1, 1; // 3 job-level chains, whose RTDA are (4, 4), (14, -1), (4, 4)

    ScheduleOptions scheduleOptions;
    scheduleOptions.causeEffectChainNumber_ = 1;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;
    scheduleOptions.freshTol_ = 0;

    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial, scheduleOptions), 32.5);
    EXPECT_EQ(RTSS21ICObj::Evaluate(dagTasks, tasksInfo, initial, scheduleOptions), 180 + 29 * 0.5);

    // EXPECT_LONGS_EQUAL(18, status.ReadObj());
    // EXPECT_DOUBLES_EQUAL(32.5, status.ObjWeighted(), 0.1);
    // EXPECT_LONGS_EQUAL(18, status.ObjBarrier());

    // EXPECT_LONGS_EQUAL(18, status2.ReadObj());
    // EXPECT_LONGS_EQUAL(18, status2.ObjBarrier());
    // EXPECT_DOUBLES_EQUAL(32.5, status2.ObjWeighted(), 0.1);
    scheduleOptions.freshTol_ = 100;
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial, scheduleOptions), 32.5);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}