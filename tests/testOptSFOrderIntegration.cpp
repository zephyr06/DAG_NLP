
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "sources/Optimization/OptimizeSFOrder.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class DAGScheduleOptimizerTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv", "orig"); // single-rate dag
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

        dagScheduleOptimizer = DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj>(dagTasks, scheduleOptions, timeLimits);
    };

    double timeLimits = 1;
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
    DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj> dagScheduleOptimizer;
};
class DAGScheduleOptimizerTest2 : public DAGScheduleOptimizerTest1
{
protected:
    void SetUp() override
    {
        DAGScheduleOptimizerTest1::SetUp();
        dagScheduleOptimizer2 = DAGScheduleOptimizer<LPOrderScheduler, RTDAExperimentObj>(dagTasks, scheduleOptions, timeLimits);
    };

    DAGScheduleOptimizer<LPOrderScheduler, RTDAExperimentObj> dagScheduleOptimizer2;
};

// TODO: add a case that evaluates initial to be schedulable
TEST_F(DAGScheduleOptimizerTest1, UpdateStatus)
{
    VectorDynamic initial = GenerateVectorDynamic(19);
    initial << 0, 199, 200, 300, 499, 511, 1, 198, 200, 399, 400, 599, 187, 201, 500, 178, 490, 107, 307;
    SFOrder jobOrder(tasksInfo, initial);
    JobGroupRange uselessRange(0, 100);
    LLint uselessFinishP = 100;
    EXPECT_EQ(0, dagScheduleOptimizer.UpdateStatus(jobOrder, uselessRange, uselessFinishP));
}

TEST(IntegrationTest, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v27.csv", "orig");

    ScheduleResult sth;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
    scheduleOption.LoadParametersYaml();
    scheduleOption.doScheduleOptimization_ = 0;
    scheduleOption.doScheduleOptimizationOnlyOnce_ = 0;
    sth = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(dagTasks, scheduleOption);
    EXPECT_THAT(sth.obj_, testing::Le(8338 * 2));
}

TEST(IntegrationTest, v2)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v28.csv", "orig");

    ScheduleResult sth;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
    scheduleOption.LoadParametersYaml();
    scheduleOption.doScheduleOptimization_ = 0;
    scheduleOption.doScheduleOptimizationOnlyOnce_ = 0;
    sth = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(dagTasks, scheduleOption);
    EXPECT_THAT(sth.obj_, testing::Le(8638 + 17638));
}

TEST_F(DAGScheduleOptimizerTest2, UpdateStatus)
{

    VectorDynamic initial = GenerateVectorDynamic(19);
    initial << 0, 199, 200, 300, 499, 511, 1, 198, 200, 399, 400, 599, 187, 201, 500, 178, 490, 107, 307;
    SFOrder jobOrder(tasksInfo, initial);
    jobOrder.print();
    PrintSchedule(tasksInfo, initial);
    JobGroupRange uselessRange(0, 100);
    LLint uselessFinishP = 100;
    std::vector<uint> processorJobVector;
    VectorDynamic startTimeVectorFromScheduler = LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVector);
    EXPECT_EQ(initial, startTimeVectorFromScheduler);
    EXPECT_GT(dagScheduleOptimizer.UpdateStatus(jobOrder, uselessRange, uselessFinishP), SFOrderStatus::Infeasible);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}