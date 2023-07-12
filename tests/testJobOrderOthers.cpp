#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
// using namespace LPOptimizer;

class RTDATest1 : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v18";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(4);
    startTimeVector << 0, 10, 1, 0;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }

  void SetUpTaskSet(std::string taskSet)
  {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + taskSet + ".csv", "orig");
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);

    
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
  std::unordered_map<JobCEC, int> jobGroupMap;

  JobCEC job0 = JobCEC(0, 0);
  JobCEC job1 = JobCEC(0, 1);
  JobCEC job2 = JobCEC(0, 2);
  JobCEC job3 = JobCEC(0, 3);
};
class RTDATest9 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v18";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(4);
    startTimeVector << 0, 10, 0, 1;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest9, order_new_constructor)
{
  EXPECT_THAT(jobOrder.GetJobStartInstancePosition((JobCEC(0, 0))),
              testing::Le(jobOrder.GetJobStartInstancePosition((JobCEC(1, 0)))));
}
TEST_F(RTDATest1, GetJobWithinHyperPeriod)
{
  JobCEC job(0, -5);
  EXPECT_EQ(1, job.GetJobWithinHyperPeriod(tasksInfo).jobId);
}

TEST_F(RTDATest1, MapJobIndexToHpInfo)
{
  JobCEC job(0, -5);
  JobIndexHelper info = MapJobIndexToHpInfo(job, tasksInfo);
  EXPECT_EQ(1, info.job_within_hp.jobId);
  EXPECT_EQ(-24, info.index_offset);
  job.jobId = -4;
  info = MapJobIndexToHpInfo(job, tasksInfo);
  EXPECT_EQ(0, info.job_within_hp.jobId);
  EXPECT_EQ(-16, info.index_offset);

  JobCEC job2(0, 5);
  info = MapJobIndexToHpInfo(job2, tasksInfo);
  EXPECT_EQ(1, info.job_within_hp.jobId);
  EXPECT_EQ(16, info.index_offset);
  job.jobId = 4;
  info = MapJobIndexToHpInfo(job, tasksInfo);
  EXPECT_EQ(0, info.job_within_hp.jobId);
  EXPECT_EQ(16, info.index_offset);

  JobCEC job3(1, -1);
  info = MapJobIndexToHpInfo(job3, tasksInfo);
  EXPECT_EQ(0, info.job_within_hp.jobId);
  EXPECT_EQ(-8, info.index_offset);

  JobCEC job4(1, 1);
  info = MapJobIndexToHpInfo(job4, tasksInfo);
  EXPECT_EQ(0, info.job_within_hp.jobId);
  EXPECT_EQ(8, info.index_offset);
}

TEST(GetStartTime, v1)
{
  using namespace OrderOptDAG_SPACE;
  DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v1.csv",
                                     "orig"); // single-rate dag
  TaskSet tasks = dagTasks.tasks;
  TaskSetInfoDerived tasksInfo(tasks);

  // VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables,
  // tasksInfo.variableDimension, initialUser);
  VectorDynamic initialEstimate = GenerateVectorDynamic(5);
  initialEstimate << 1, 2, 3, 4, 5;

  EXPECT_EQ(1, GetStartTime({0, 0}, initialEstimate, tasksInfo));
  EXPECT_EQ(11, GetFinishTime({0, 0}, initialEstimate, tasksInfo));
  EXPECT_EQ(201, GetStartTime({0, 1}, initialEstimate, tasksInfo));
  EXPECT_EQ(401, GetStartTime({0, 2}, initialEstimate, tasksInfo));
  EXPECT_EQ(403, GetStartTime({2, 2}, initialEstimate, tasksInfo));
  EXPECT_EQ(405, GetStartTime({4, 2}, initialEstimate, tasksInfo));

  EXPECT_EQ(1 - 200, GetStartTime({0, -1}, initialEstimate, tasksInfo));
  EXPECT_EQ(1 - 400, GetStartTime({0, -2}, initialEstimate, tasksInfo));
}
TEST(GetStartTime, v2)
{
  using namespace OrderOptDAG_SPACE;
  DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v37.csv",
                                     "orig");
  TaskSet tasks = dagTasks.tasks;
  TaskSetInfoDerived tasksInfo(tasks);
  VectorDynamic initialEstimate = GenerateVectorDynamic(7);
  initialEstimate << 0, 10, 20, 0, 10, 20, 5;

  EXPECT_EQ(20 - 30, GetStartTime({0, -1}, initialEstimate, tasksInfo));
  EXPECT_EQ(10 - 30, GetStartTime({0, -2}, initialEstimate, tasksInfo));
  EXPECT_EQ(0 - 30, GetStartTime({0, -3}, initialEstimate, tasksInfo));
  EXPECT_EQ(20 - 60, GetStartTime({0, -4}, initialEstimate, tasksInfo));
}
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}