

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/IndependentAnalysis.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"
#include <functional>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

class RTDATest_n5_v84 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string taskSetName = "test_n5_v84";
    SetUpTaskSet(taskSetName);
    startTimeVector = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
  }

  void SetUpTaskSet(std::string taskSet)
  {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + taskSet + ".csv", "orig");
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);

    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 1e6;
    scheduleOptions.sensorFusionTolerance_ = 1e6;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 1000;
    scheduleOptions.processorNum_ = 2;

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

TEST_F(RTDATest_n5_v84, WhetherImmediateForwardAdjacent_v1)
{
  PrintSchedule(tasksInfo, startTimeVector);
  auto jobOrderRef = jobOrder;
  auto longestJobChains_ = LongestCAChain(dagTasks, tasksInfo, jobOrderRef, startTimeVector,
                                          scheduleOptions.processorNum_);
  auto jobGroupMap_ = ExtractIndependentJobGroups(jobOrderRef, tasksInfo);
  auto centralJob = FindCentralJobs(longestJobChains_, tasksInfo);
  std::vector<JobCEC> forwardJobs = FindForwardAdjacentJob(JobCEC(0, 1), jobOrderRef, tasksInfo, startTimeVector);
  EXPECT_EQ(3, forwardJobs.size());
  // Note: the order in the following test doesn't matter
  // EXPECT_EQ(JobCEC(0, 1), forwardJobs[0]);
  // EXPECT_EQ(JobCEC(1, 1), forwardJobs[1]);
  // EXPECT_EQ(JobCEC(3, 1), forwardJobs[2]);
  EXPECT_TRUE(ifExist(JobCEC(0, 1), forwardJobs));
  EXPECT_TRUE(ifExist(JobCEC(1, 1), forwardJobs));
  EXPECT_TRUE(ifExist(JobCEC(3, 1), forwardJobs));
  // auto activeJobs_ = FindActiveJobs(centralJob, jobOrderRef, tasksInfo, startTimeVector);
  // auto sth = FindForwardAdjacentJob(JobCEC(0, 0), jobOrderRef, tasksInfo, startTimeVector);
  // sth = FindBackwardAdjacentJob(JobCEC(4, 0), jobOrderRef, tasksInfo, startTimeVector);
  // sth = FindBackwardAdjacentJob(JobCEC(4, 1), jobOrderRef, tasksInfo, startTimeVector);
}
class RTDATest_n5_v85 : public RTDATest_n5_v84
{
  void SetUp() override
  {
    std::string taskSetName = "test_n5_v85";
    SetUpTaskSet(taskSetName);
    startTimeVector = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
  }
};
TEST_F(RTDATest_n5_v85, WhetherImmediateForwardAdjacent_v1)
{
  startTimeVector << 998.001,
      3831,
      998,
      3000,
      0.001,
      2256,
      5002,
      1274,
      3831,
      0,
      2833,
      5002;
  // TODO: analyze the longest DA job chain;
}
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}