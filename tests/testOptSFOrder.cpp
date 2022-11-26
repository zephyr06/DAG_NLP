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
        std::string taskSetName = "test_n3_v18";
        SetUpTaskSet(taskSetName);
        startTimeVector = GenerateVectorDynamic(4);
        startTimeVector << 0, 10, 1, 1;
        sfOrder = SFOrder(tasksInfo, startTimeVector);
        sfOrder.print();
    }

    void SetUpTaskSet(std::string taskSet)
    {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + taskSet + ".csv", "orig"); // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 2};
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
    VectorDynamic startTimeVector;
    SFOrder sfOrder;
    std::vector<uint> processorJobVec;
};
TEST_F(ScheduleDAGModelTest1, FindJobActivateRange)
{
    JobCEC jobRelocate(0, 0);
    JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
    EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
    EXPECT_EQ(7, jobStartFinishInstActiveRange.maxIndex);
}

TEST_F(ScheduleDAGModelTest1, initialEstimatePool)
{
    VectorDynamic initialBase = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);

    VectorDynamic initialFromPool = SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo, scheduleOptions);

    EXPECT_LT(RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialFromPool, scheduleOptions), RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialBase, scheduleOptions));
}

class ScheduleDAGModelTest2 : public ScheduleDAGModelTest1
{
protected:
    void SetUp() override
    {
        std::string taskSetName = "test_n3_v29";
        SetUpTaskSet(taskSetName);
    }
};

TEST_F(ScheduleDAGModelTest2, SelectInitialFromPool_verify_feasibility)
{
    auto initialSTV = SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo, scheduleOptions, processorJobVec);
    EXPECT_TRUE(ExamBasic_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec, scheduleOptions.processorNum_));
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}