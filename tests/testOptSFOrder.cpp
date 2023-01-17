#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include <gtest/gtest.h>
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
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + taskSet + ".csv",
                             "orig"); // single-rate dag
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
    chain1 = {0, 2};
    dagTasks.chains_[0] = chain1;

    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;

    scheduleOptions.selectInitialFromPoolCandidate_ = 100;
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
  JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
  EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
  EXPECT_EQ(7, jobStartFinishInstActiveRange.maxIndex);
}

TEST_F(ScheduleDAGModelTest1, initialEstimatePool) {
  VectorDynamic initialBase = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);

  VectorDynamic initialFromPool =
      SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo, scheduleOptions);

  EXPECT_LT(RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialFromPool, scheduleOptions),
            RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, initialBase, scheduleOptions));
}

class ScheduleDAGModelTest2 : public ScheduleDAGModelTest1 {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v29";
    SetUpTaskSet(taskSetName);
  }
};

TEST_F(ScheduleDAGModelTest2, SelectInitialFromPool_verify_feasibility) {
  auto initialSTV =
      SelectInitialFromPool<RTDAExperimentObj>(dagTasks, tasksInfo, scheduleOptions, processorJobVec);
  EXPECT_TRUE(
      ExamBasic_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec, scheduleOptions.processorNum_));
}

class ScheduleDAGModelTest3 : public ScheduleDAGModelTest1 {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v32";
    SetUpTaskSet(taskSetName);
  }
};
TEST_F(ScheduleDAGModelTest3, ScheduleDAGLS_LFT_true) {
  ScheduleResult res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT<RTDAExperimentObj>(
      dagTasks, scheduleOptions, dagTasks.GetSfBound(), dagTasks.GetRtdaBound());
  EXPECT_TRUE(res.schedulable_);
}

TEST_F(ScheduleDAGModelTest3, ScheduleDAGLS_LFT_false) {
  ScheduleResult res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT<RTSS21ICObj>(
      dagTasks, scheduleOptions, dagTasks.GetSfBound(), dagTasks.GetRtdaBound());
  EXPECT_FALSE(res.schedulable_);
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
        ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
    sfOrder = SFOrder(tasksInfo, startTimeVector);
    sfOrder.print();
  }
};
TEST_F(ScheduleDAGModelTest4, FindJobActivateRange) {
  JobCEC jobRelocate(0, 0);
  JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
  EXPECT_EQ(0, jobStartFinishInstActiveRange.minIndex);
  EXPECT_EQ(5 + 1, jobStartFinishInstActiveRange.maxIndex);
}

TEST_F(ScheduleDAGModelTest4, FindJobActivateRange_min) {
  JobCEC jobRelocate(0, 1);
  JobGroupRange jobStartFinishInstActiveRange = FindJobActivateRange(jobRelocate, sfOrder, tasksInfo);
  EXPECT_EQ(4, jobStartFinishInstActiveRange.minIndex);
  EXPECT_EQ(10 + 1, jobStartFinishInstActiveRange.maxIndex);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}