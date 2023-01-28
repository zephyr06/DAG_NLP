
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"
#include <functional>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
// TODO: consider analyze another task set
class RTDATest1 : public ::testing::Test {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v18";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(4);
    startTimeVector << 0, 10, 1, 0;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
  }

  void SetUpTaskSet(std::string taskSet) {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + taskSet + ".csv", "orig");
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);

    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 1e6;
    scheduleOptions.sensorFusionTolerance_ = 1e6;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 1000;

    scheduleOptions.selectInitialFromPoolCandidate_ = 0; // 1000 if adding select initial from pool
  }
  DAG_Model dagTasks;
  TaskSet tasks;
  TaskSetInfoDerived tasksInfo;
  std::vector<int> chain1;
  ScheduleOptions scheduleOptions;
  VectorDynamic startTimeVector;
  SFOrder jobOrder;
  std::vector<uint> processorJobVec;

  JobCEC job0 = JobCEC(0, 0);
  JobCEC job1 = JobCEC(0, 1);
  JobCEC job2 = JobCEC(0, 2);
  JobCEC job3 = JobCEC(0, 3);
};

TEST_F(RTDATest1, GetReactionTime) {
  auto react_chain_map = GetReactionChainMap(dagTasks, tasksInfo, jobOrder, scheduleOptions.processorNum_,
                                             dagTasks.chains_[0], 0);
  auto chain0 = react_chain_map.at(job0);
  EXPECT_EQ(23, GetReactionTime(chain0, startTimeVector, tasksInfo));
  // EXPECT_EQ(-1, GetDataAge(chain0, startTimeVector, tasksInfo));
  auto chain1 = react_chain_map.at(job1);
  auto prevChain1 = react_chain_map.at(job0);
  EXPECT_EQ(13, GetReactionTime(chain1, startTimeVector, tasksInfo));
  EXPECT_EQ(-1, GetDataAge(chain1, prevChain1, startTimeVector, tasksInfo));
  auto chain2 = react_chain_map.at(job2);
  auto prevChain2 = react_chain_map.at(job1);
  EXPECT_EQ(23, GetReactionTime(chain2, startTimeVector, tasksInfo));
  EXPECT_EQ(13, GetDataAge(chain2, prevChain2, startTimeVector, tasksInfo));
  auto chain3 = react_chain_map.at(job3);
  auto prevChain3 = react_chain_map.at(job2);
  EXPECT_EQ(13, GetReactionTime(chain3, startTimeVector, tasksInfo));
  EXPECT_EQ(-1, GetDataAge(chain3, prevChain3, startTimeVector, tasksInfo));
}

TEST_F(RTDATest1, longest_chain) {

  auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], startTimeVector);

  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);
  // order:
  // std::vector<std::vector<JobCEC>> longestChains;
  // longestChains.reserve(3);
  std::vector<JobCEC> chain0 = {JobCEC(0, 0), JobCEC(2, 1)}; // RT
  std::vector<JobCEC> chain2 = {JobCEC(0, 2), JobCEC(2, 2)}; // RT&DA
  EXPECT_EQ(2, longestChain.longestChains_.size());
  AssertEqualVectorNoRepeat<JobCEC>(chain2, longestChain.longestChains_[0], 0, __LINE__);
  AssertEqualVectorNoRepeat<JobCEC>(chain0, longestChain.longestChains_[1], 0, __LINE__);
}

TEST_F(RTDATest1, break_chain) {

  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);

  EXPECT_TRUE(WhetherJobBreakChain(job0, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job0, 0, 1, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job2, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job1, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobBreakChain(JobCEC(1, 0), 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
}

double GetJobMaxStartTime(const JobCEC &jobCurr, SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo) {
  double maxStartTime = GetDeadline(jobCurr, tasksInfo) - GetExecutionTime(jobCurr, tasksInfo);
  for (LLint i = jobOrder.GetJobStartInstancePosition(jobCurr) + 1; i < jobOrder.size(); i++) {
    TimeInstance instCurr = jobOrder[i];
    double instCurrMaxStart = instCurr.GetRangeMax(tasksInfo);
    if (instCurrMaxStart < maxStartTime) {
      maxStartTime = instCurrMaxStart;
    }
    double instCurrMinStart = instCurr.GetRangeMin(tasksInfo);
    if (maxStartTime < instCurrMinStart)
      break;
  }
  return maxStartTime;
}

bool WhetherJobStartLater(const JobCEC &jobCurr, SFOrder &jobOrderOrg, SFOrder &jobOrderNew,
                          const TaskSetInfoDerived &tasksInfo) {

  double maxStartTimeOrg = GetJobMaxStartTime(jobCurr, jobOrderOrg, tasksInfo);
  double maxStartTimeNew = GetJobMaxStartTime(jobCurr, jobOrderNew, tasksInfo);
  return maxStartTimeNew > maxStartTimeOrg;
}
bool WhetherJobStartLater(const JobCEC &jobCurr, const JobCEC &jobChanged, LLint startP, LLint finishP,
                          SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo) {

  SFOrder jobOrderNew = jobOrder;
  jobOrderNew.RemoveJob(jobChanged);
  jobOrderNew.InsertStart(jobChanged, startP);
  jobOrderNew.InsertFinish(jobChanged, finishP);
  return WhetherJobStartLater(jobCurr, jobOrder, jobOrderNew, tasksInfo);
}
TEST_F(RTDATest1, GetJobMinStartTime) {
  EXPECT_EQ(0, GetJobMinStartTime(JobCEC(0, 0), jobOrder, tasksInfo));
  EXPECT_EQ(0, GetJobMinStartTime(JobCEC(2, 0), jobOrder, tasksInfo));
  EXPECT_EQ(1, GetJobMinStartTime(JobCEC(1, 0), jobOrder, tasksInfo));
  EXPECT_EQ(10, GetJobMinStartTime(JobCEC(0, 1), jobOrder, tasksInfo));
}
TEST_F(RTDATest1, GetJobMaxStartTime) {
  EXPECT_EQ(9, GetJobMaxStartTime(JobCEC(0, 0), jobOrder, tasksInfo));
  EXPECT_EQ(10, GetJobMaxStartTime(JobCEC(2, 0), jobOrder, tasksInfo));
  EXPECT_EQ(11, GetJobMaxStartTime(JobCEC(1, 0), jobOrder, tasksInfo));
  EXPECT_EQ(19, GetJobMaxStartTime(JobCEC(0, 1), jobOrder, tasksInfo));
}

TEST_F(RTDATest1, WhetherJobStartEarlier) {
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 1), JobCEC(0, 1), 6, 7, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 1), JobCEC(1, 0), 6, 7, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 1), JobCEC(2, 0), 6, 7, jobOrder, tasksInfo));

  EXPECT_TRUE(WhetherJobStartEarlier(JobCEC(1, 0), JobCEC(0, 0), 4, 5, jobOrder, tasksInfo));
}

class RTDATest2 : public RTDATest1 {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v37";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(7);
    startTimeVector << 0, 10, 20, 0, 10, 20, 3;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    dagTasks.chains_[0] = {0, 2};
    dagTasks.chains_.push_back({0, 1, 2});
  }
};

TEST_F(RTDATest2, GetJobMinStartTime) {

  EXPECT_EQ(0, GetJobMinStartTime(JobCEC(0, 0), jobOrder, tasksInfo));
  EXPECT_EQ(10, GetJobMinStartTime(JobCEC(0, 1), jobOrder, tasksInfo));
  EXPECT_EQ(20, GetJobMinStartTime(JobCEC(0, 2), jobOrder, tasksInfo));

  EXPECT_EQ(0, GetJobMinStartTime(JobCEC(1, 0), jobOrder, tasksInfo));
  EXPECT_EQ(10, GetJobMinStartTime(JobCEC(1, 1), jobOrder, tasksInfo));
  EXPECT_EQ(20, GetJobMinStartTime(JobCEC(1, 2), jobOrder, tasksInfo));

  EXPECT_EQ(2, GetJobMinStartTime(JobCEC(2, 0), jobOrder, tasksInfo));
}

TEST_F(RTDATest2, break_chain) {

  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);
  JobCEC job10(1, 0);
  JobCEC job11(1, 1);
  JobCEC job12(1, 2);
  EXPECT_TRUE(WhetherJobBreakChain(job10, 1, 2, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobBreakChain(job10, 0, 1, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job11, 1, 2, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobBreakChain(job12, 10, 11, longestChain, dagTasks, jobOrder, tasksInfo));
}

TEST_F(RTDATest2, WhetherJobStartEarlier) {
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(0, 1), 6, 7, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 1), 6, 7, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(2, 0), 6, 7, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 2), 6, 7, jobOrder, tasksInfo));

  EXPECT_FALSE(WhetherJobStartEarlier(JobCEC(2, 0), JobCEC(0, 0), 4, 5, jobOrder, tasksInfo));
}

class RTDATest3 : public RTDATest1 {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v37";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(7);
    startTimeVector << 0, 16, 20, 0, 10, 20, 13;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    dagTasks.chains_[0] = {0, 2};
    dagTasks.chains_.push_back({0, 1, 2});
  }
};
TEST_F(RTDATest3, WhetherJobStartEarlier) {

  EXPECT_TRUE(WhetherJobStartEarlier(JobCEC(2, 0), JobCEC(1, 1), 10, 11, jobOrder, tasksInfo));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}