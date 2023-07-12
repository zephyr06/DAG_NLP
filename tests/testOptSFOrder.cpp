#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Utils/OptimizeOrderUtils.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class ScheduleDAGModelTest1 : public ::testing::Test {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v18";
        SetUpTaskSet(taskSetName);
        startTimeVector = GenerateVectorDynamic(4);
        startTimeVector << 0, 10, 1, 1;
        sfOrder = SFOrder(tasksInfo, startTimeVector);
        sfOrder.print();
    }

    void SetUpTaskSet(std::string taskSet) {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +
                                     "TaskData/" + taskSet + ".csv",
                                 "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 2};
        dagTasks.chains_[0] = chain1;

        
        scheduleOptions.freshTol_ = 1e6;
        scheduleOptions.sensorFusionTolerance_ = 1e6;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 1000;

        scheduleOptions.selectInitialFromPoolCandidate_ =
            0;  // 1000 if adding select initial from pool
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
TEST_F(ScheduleDAGModelTest1, FindJobActivateRange) {
    JobCEC jobRelocate(0, 0);
    JobGroupRange jobStartFinishInstActiveRange =
        FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
    EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
    EXPECT_EQ(6, jobStartFinishInstActiveRange.maxIndex);
}

// TEST_F(ScheduleDAGModelTest1, initialEstimatePool) {
//   VectorDynamic initialBase = ListSchedulingLFTPA(dagTasks, tasksInfo,
//   scheduleOptions.processorNum_);

//   VectorDynamic initialFromPool =
//       SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo,
//       scheduleOptions);

//   EXPECT_LT(RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialFromPool,
//   scheduleOptions),
//             RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialBase,
//             scheduleOptions));
// }

class ScheduleDAGModelTest2 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v29";
        SetUpTaskSet(taskSetName);
    }
};

// TEST_F(ScheduleDAGModelTest2, SelectInitialFromPool_verify_feasibility) {
//   auto initialSTV =
//       SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo,
//       scheduleOptions, processorJobVec);
//   EXPECT_TRUE(
//       ExamBasic_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec,
//       scheduleOptions.processorNum_));
// }

class ScheduleDAGModelTest3 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v32";
        SetUpTaskSet(taskSetName);
    }
};
TEST_F(ScheduleDAGModelTest3, ScheduleDAGLS_LFT_true) {
    ScheduleResult res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT<ReactionTimeObj>(
        dagTasks, scheduleOptions);
    EXPECT_TRUE(res.schedulable_);
}

class ScheduleDAGModelTest4 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v29";
        SetUpTaskSet(taskSetName);

        const TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::vector<uint> processorJobVec;
        startTimeVector =
            ListSchedulingLFTPA(dagTasks, tasksInfo,
                                scheduleOptions.processorNum_, processorJobVec);
        sfOrder = SFOrder(tasksInfo, startTimeVector);
        sfOrder.print();
    }
};
TEST_F(ScheduleDAGModelTest4, FindJobActivateRange) {
    JobCEC jobRelocate(0, 0);
    JobGroupRange jobStartFinishInstActiveRange =
        FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
    EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
    EXPECT_EQ(5, jobStartFinishInstActiveRange.maxIndex);
}

TEST_F(ScheduleDAGModelTest4, FindJobActivateRange_min) {
    JobCEC jobRelocate(0, 1);
    JobGroupRange jobStartFinishInstActiveRange =
        FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
    EXPECT_EQ(4, jobStartFinishInstActiveRange.minIndex);
    EXPECT_EQ(10, jobStartFinishInstActiveRange.maxIndex);
}
class ScheduleDAGModelTest5 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v33";
        SetUpTaskSet(taskSetName);
        chain1 = {2, 0};
        dagTasks.chains_[0] = chain1;
        scheduleOptions.processorNum_ = 2;

        const TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::vector<uint> processorJobVec;
        startTimeVector =
            ListSchedulingLFTPA(dagTasks, tasksInfo,
                                scheduleOptions.processorNum_, processorJobVec);
        startTimeVector << 1000, 2325, 4325, 6325, 8650, 0, 1000, 2000, 3222,
            4000, 5222, 6000, 7222, 8650, 9547, 1897;
        sfOrder = SFOrder(tasksInfo, startTimeVector);
        auto instTemp = sfOrder.instanceOrder_[2];
        sfOrder.instanceOrder_[2] = sfOrder.instanceOrder_[3];
        sfOrder.instanceOrder_[3] = instTemp;
        sfOrder.print();
    }
};
TEST_F(ScheduleDAGModelTest5, assign_processor) {
    EXPECT_TRUE(ProcessorAssignment::AssignProcessor(
        tasksInfo, sfOrder, scheduleOptions.processorNum_, processorJobVec));
    JobCEC jobCurr(0, 3);
    sfOrder.RemoveJob(jobCurr);
    sfOrder.InsertStart(jobCurr, 21);
    sfOrder.InsertFinish(jobCurr, 23);
    sfOrder.print();

    EXPECT_FALSE(ProcessorAssignment::AssignProcessor(
        tasksInfo, sfOrder, scheduleOptions.processorNum_, processorJobVec));

    EXPECT_FALSE(ProcessorAssignment::AssignProcessor(
        tasksInfo, sfOrder, scheduleOptions.processorNum_, processorJobVec));

    startTimeVector = SimpleOrderScheduler::schedule(
        dagTasks, tasksInfo, scheduleOptions, sfOrder, processorJobVec);
    bool schedulable =
        ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector,
                              processorJobVec, scheduleOptions.processorNum_);
    EXPECT_FALSE(schedulable);
}

class ScheduleDAGModelTest6 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n3_v34";
        SetUpTaskSet(taskSetName);
        chain1 = {0, 2};
        dagTasks.chains_[0] = chain1;
        scheduleOptions.processorNum_ = 2;

        const TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::vector<uint> processorJobVec;
        // startTimeVector =
        //     ListSchedulingLFTPA(dagTasks, tasksInfo,
        //     scheduleOptions.processorNum_, processorJobVec);
        // startTimeVector << 8, 10, 1, 3;
        // sfOrder = SFOrder(tasksInfo, startTimeVector);
        // sfOrder.print();

        dagScheduleOptimizer =
            DAGScheduleOptimizer<LPOrderScheduler, ReactionTimeObj>(
                dagTasks, scheduleOptions, 100);

        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks, tasksInfo, scheduleOptions.processorNum_);
        initialSTV(10) = 6;
        dagScheduleOptimizer.jobOrderRef = SFOrder(tasksInfo, initialSTV);
        dagScheduleOptimizer.statusPrev =
            IterationStatus<LPOrderScheduler, ReactionTimeObj>(
                dagTasks, tasksInfo, dagScheduleOptimizer.jobOrderRef,
                scheduleOptions);
    }

    DAGScheduleOptimizer<LPOrderScheduler, ReactionTimeObj>
        dagScheduleOptimizer;
};

TEST_F(ScheduleDAGModelTest6, ImproveJobOrderPerJob_Single_update) {
    JobCEC jobRelocate(2, 0);
    dagScheduleOptimizer.ImproveJobOrderPerJob(jobRelocate);
    EXPECT_EQ(1, dagScheduleOptimizer.countMakeProgress);
    EXPECT_THAT(dagScheduleOptimizer.countIterationStatus, testing::Ge(1));
}

class ScheduleDAGModelTest7 : public ScheduleDAGModelTest1 {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n30_v1";
        SetUpTaskSet(taskSetName);
        scheduleOptions.processorNum_ = 2;

        const TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::vector<uint> processorJobVec;
        VectorDynamic initial =
            ListSchedulingLFTPA(dagTasks, tasksInfo,
                                scheduleOptions.processorNum_, processorJobVec);

        sfOrder = SFOrder(tasksInfo, initial);
        // sfOrder.print();
    }
};

// TEST_F(ScheduleDAGModelTest7, warm_start_LP) {
//   BeginTimer("main");
//   BeginTimer("Direct_LP");
//   auto schedule0 = LPOrderScheduler::schedule(dagTasks, tasksInfo,
//   scheduleOptions, sfOrder, processorJobVec); EndTimer("Direct_LP");
//   BeginTimer("Warm_start_LP");
//   auto schedule1 =
//       LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions,
//       sfOrder, processorJobVec, schedule0);
//   EndTimer("Warm_start_LP");
//   EXPECT_TRUE(gtsam::assert_equal(schedule0, schedule1));
//   EndTimer("main");
//   PrintTimer();
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}