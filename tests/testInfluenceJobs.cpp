

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Optimization/IndependentAnalysis.h"
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"
#include <functional>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

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
  std::unordered_map<JobCEC, int> jobGroupMap;

  JobCEC job0 = JobCEC(0, 0);
  JobCEC job1 = JobCEC(0, 1);
  JobCEC job2 = JobCEC(0, 2);
  JobCEC job3 = JobCEC(0, 3);
};

class RTDATest7 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 4592, 0, 2000, 4680, 6000, 8000, 782, 6000;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest7, WhetherImmediateForwardAdjacent_v1)
{
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('f', JobCEC(1, 0)),
                                               TimeInstance('s', JobCEC(2, 0)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_TRUE(WhetherImmediateForwardAdjacent(TimeInstance('s', JobCEC(2, 0)),
                                              TimeInstance('f', JobCEC(1, 0)), tasksInfo, startTimeVector,
                                              jobOrder));
  EXPECT_TRUE(WhetherImmediateForwardAdjacent(TimeInstance('s', JobCEC(0, 0)),
                                              TimeInstance('f', JobCEC(2, 0)), tasksInfo, startTimeVector,
                                              jobOrder));
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('f', JobCEC(2, 0)),
                                               TimeInstance('s', JobCEC(0, 0)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('s', JobCEC(1, 3)),
                                               TimeInstance('s', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_TRUE(WhetherImmediateForwardAdjacent(TimeInstance('s', JobCEC(2, 1)),
                                              TimeInstance('s', JobCEC(1, 3)), tasksInfo, startTimeVector,
                                              jobOrder));
  startTimeVector(5) = 9028;
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('f', JobCEC(1, 4)),
                                               TimeInstance('f', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('s', JobCEC(1, 4)),
                                               TimeInstance('f', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_FALSE(WhetherImmediateForwardAdjacent(TimeInstance('f', JobCEC(2, 1)),
                                               TimeInstance('s', JobCEC(1, 4)), tasksInfo, startTimeVector,
                                               jobOrder));
}

TEST_F(RTDATest7, WhetherImmediateAdjacent_v1)
{
  EXPECT_TRUE(WhetherImmediateAdjacent(TimeInstance('f', JobCEC(1, 0)), TimeInstance('s', JobCEC(2, 0)),
                                       tasksInfo, startTimeVector));
  EXPECT_TRUE(WhetherImmediateAdjacent(TimeInstance('s', JobCEC(2, 0)), TimeInstance('f', JobCEC(1, 0)),
                                       tasksInfo, startTimeVector));
  EXPECT_TRUE(WhetherImmediateAdjacent(TimeInstance('f', JobCEC(2, 0)), TimeInstance('s', JobCEC(0, 0)),
                                       tasksInfo, startTimeVector));
  EXPECT_TRUE(WhetherImmediateAdjacent(TimeInstance('s', JobCEC(1, 3)), TimeInstance('s', JobCEC(2, 1)),
                                       tasksInfo, startTimeVector));
  startTimeVector(5) = 9028;
  EXPECT_TRUE(WhetherImmediateAdjacent(TimeInstance('f', JobCEC(1, 4)), TimeInstance('f', JobCEC(2, 1)),
                                       tasksInfo, startTimeVector));
  EXPECT_FALSE(WhetherImmediateAdjacent(TimeInstance('s', JobCEC(1, 4)), TimeInstance('f', JobCEC(2, 1)),
                                        tasksInfo, startTimeVector));
  EXPECT_FALSE(WhetherImmediateAdjacent(TimeInstance('f', JobCEC(2, 1)), TimeInstance('s', JobCEC(1, 4)),
                                        tasksInfo, startTimeVector));
}

TEST_F(RTDATest7, FindForwardAdjacentJob)
{
  auto prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(2, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 0) == prevAdjacentJobs[0]);
  EXPECT_TRUE(JobCEC(2, 0) == prevAdjacentJobs[1]);

  prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(2, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 0) == prevAdjacentJobs[0]);

  startTimeVector(5) = 9028;
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  jobOrder.print();
  prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(2, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(2, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 4) == prevAdjacentJobs[0]);
  EXPECT_TRUE(JobCEC(1, 3) == prevAdjacentJobs[1]);
}

TEST_F(RTDATest7, WhetherJobStartEarlier_v1)
{
  // LLint startP = 0;
  // LLint finishP = 2;

  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));

  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest7, WhetherJobStartEarlier_v2)
{
  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}
// *****************************************************************************************************************

TEST_F(RTDATest7, WhetherImmediateBackwardAdjacent_v1)
{
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('f', JobCEC(1, 1)),
                                                TimeInstance('s', JobCEC(2, 0)), tasksInfo, startTimeVector,
                                                jobOrder));
  EXPECT_TRUE(WhetherImmediateBackwardAdjacent(TimeInstance('f', JobCEC(2, 0)),
                                               TimeInstance('s', JobCEC(0, 0)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_TRUE(WhetherImmediateBackwardAdjacent(TimeInstance('s', JobCEC(1, 3)),
                                               TimeInstance('s', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('s', JobCEC(2, 1)),
                                                TimeInstance('s', JobCEC(1, 3)), tasksInfo, startTimeVector,
                                                jobOrder));
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('s', JobCEC(2, 0)),
                                                TimeInstance('f', JobCEC(1, 1)), tasksInfo, startTimeVector,
                                                jobOrder));
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('s', JobCEC(2, 1)),
                                                TimeInstance('s', JobCEC(1, 3)), tasksInfo, startTimeVector,
                                                jobOrder));
  startTimeVector(5) = 9218;
  startTimeVector(7) = 5408;
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  jobOrder.print();
  EXPECT_TRUE(WhetherImmediateBackwardAdjacent(TimeInstance('f', JobCEC(2, 1)),
                                               TimeInstance('s', JobCEC(1, 4)), tasksInfo, startTimeVector,
                                               jobOrder));
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('f', JobCEC(1, 4)),
                                                TimeInstance('f', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                                jobOrder));
  EXPECT_FALSE(WhetherImmediateBackwardAdjacent(TimeInstance('s', JobCEC(1, 4)),
                                                TimeInstance('f', JobCEC(2, 1)), tasksInfo, startTimeVector,
                                                jobOrder));
}

class RTDATest8 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 5320, 0, 2000, 4626, 6000, 9218, 782, 5408;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    PrintSchedule(tasksInfo, startTimeVector);
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};

TEST_F(RTDATest8, FindBackwardAdjacentJob)
{

  auto prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(2, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 4) == prevAdjacentJobs[0]);

  prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(3, prevAdjacentJobs.size());
  // EXPECT_TRUE(JobCEC(1, 2) == prevAdjacentJobs[0]);
  EXPECT_TRUE(ifExist(JobCEC(1, 2), prevAdjacentJobs));
  // EXPECT_TRUE(JobCEC(2, 1) == prevAdjacentJobs[1]);
  EXPECT_TRUE(ifExist(JobCEC(2, 1), prevAdjacentJobs));
  // EXPECT_TRUE(JobCEC(1, 4) == prevAdjacentJobs[2]);
  EXPECT_TRUE(ifExist(JobCEC(1, 4), prevAdjacentJobs));
}

TEST_F(RTDATest8, WhetherJobStartLater)
{
  // start time of (2,0) and (1,3) doesn't reach its limits, therefore should not be analyzed
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(1, 3), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));

  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(1, 4), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}
TEST_F(RTDATest8, WhetherJobStartLater_v2)
{
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(0, 0), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(1, 2), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_TRUE(
      WhetherJobStartLater(JobCEC(1, 0), JobCEC(2, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest8, WhetherJobStartLater_v3)
{
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  startTimeVector << 3810, 0, 2000, 4000, 6000, 8000, 0, 6000;
  // this function doesn't work with SimpleOrderScheduler
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  // EXPECT_TRUE(
  //     WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

class RTDATest9 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 3810, 0, 2000, 4000, 7218, 9218, 0, 6190;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest9, WhetherJobStartLater_v4)
{
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 0), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest9, FindBackwardAdjacentJob)
{
  auto prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(1, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(0, prevAdjacentJobs.size());
}

TEST_F(RTDATest8, LP_Optimality)
{
  startTimeVector << 3810, 0, 2000, 4000, 7218, 9218, 0, 6190;
  std::cout << "Old rtda obj: "
            << RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeVector, scheduleOptions) << "\n";
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  jobOrder.print();
  startTimeVector << 3811, 0, 2000, 4000, 7218, 9218, 1, 6190;
  std::cout << "new rtda obj: "
            << RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeVector, scheduleOptions) << "\n";
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  jobOrder.print();
}

TEST_F(RTDATest8, FindCentralForwardJob)
{
  std::vector<JobCEC> centralSourceJob = {JobCEC(2, 1)};
  std::vector<JobCEC> centralSinkJob = {JobCEC(0, 0)};
  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_, "ReactionTimeObj");
  auto centralJobActual = FindCentralJobs(longestChain, tasksInfo);
  AssertEqualVectorNoRepeat<JobCEC>(centralSourceJob, centralJobActual.backwardJobs, 1e-3, __LINE__);
  AssertEqualVectorNoRepeat<JobCEC>(centralSinkJob, centralJobActual.forwardJobs, 1e-3, __LINE__);
}

TEST_F(RTDATest8, FindActiveJobs)
{
  std::vector<JobCEC> activeJob = {JobCEC(0, 0), JobCEC(1, 4), JobCEC(2, 1)};
  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_, "ReactionTimeObj");
  CentralJobs centralJob = FindCentralJobs(longestChain, tasksInfo);

  auto activeJobActual = FindActiveJobs(centralJob, jobOrder, tasksInfo, startTimeVector);
  AssertEqualVectorNoRepeat<JobCEC>(activeJob, activeJobActual.GetJobs(), 1e-3, __LINE__);
}

class RTDATest10 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v44";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(16);
    startTimeVector << 0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 994, 0, 2000, 4000, 6000,
        8000;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest10, FindBackwardAdjacentJob)
{
  auto prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(0, 3), jobOrder, tasksInfo, startTimeVector);
  // EXPECT_EQ(0, prevAdjacentJobs.size());
}
// TEST_F(RTDATest10, FindDirection) {
//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_); CentralJobs centralJob = FindCentralJobs(longestChain, tasksInfo); auto
//   activeJobActual = FindActiveJobs(centralJob, jobOrder, tasksInfo, startTimeVector);
//   EXPECT_TRUE(activeJobActual.FindDirection(JobCEC(0, 0)));
//   EXPECT_FALSE(activeJobActual.FindDirection(JobCEC(2, 0)));
//   EXPECT_FALSE(activeJobActual.FindDirection(JobCEC(2, 1)));
// }
class RTDATest11 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v46";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 4592, 0, 2000, 5218, 6000, 9218, 782, 6190;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest11, FindForwardAdjacentJob)
{
  auto prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(2, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
}
TEST_F(RTDATest11, FindForwardAdjacentJob_v2)
{
  auto prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(2, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(2, 0) == *std::find(prevAdjacentJobs.begin(), prevAdjacentJobs.end(), JobCEC(2, 0)));
}

class RTDATest12 : public RTDATest1
{
  void SetUp() override
  {
    std::string taskSetName = "test_n3_v48";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(16);
    startTimeVector << 4103, 0, 1000, 2402, 3000, 4000, 5598, 6402, 7000, 8402, 9000, 402, 2402, 4103, 6402,
        8402;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};

TEST_F(RTDATest12, IA_permute_job_order)
{
  VectorDynamic schedule1 =
      LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec, "ReactionTimeObj");
  double obj = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, schedule1, scheduleOptions);

  SFOrder jobOrder2 = jobOrder;
  JobCEC jobCurr = JobCEC(2, 0);
  jobOrder2.RemoveJob(jobCurr);
  jobOrder2.InsertStart(jobCurr, 2);
  jobOrder2.InsertFinish(jobCurr, 4);
  VectorDynamic schedule3 =
      LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder2, processorJobVec, "ReactionTimeObj");
  double obj3 = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, schedule3, scheduleOptions);
  EXPECT_EQ(obj, obj3);
}

TEST_F(RTDATest12, SFOrder_order_same_time)
{
  jobOrder.print();
  EXPECT_THAT(jobOrder.GetJobFinishInstancePosition(JobCEC(1, 4)),
              testing::Le(jobOrder.GetJobStartInstancePosition(JobCEC(0, 0))));
  EXPECT_THAT(jobOrder.GetJobStartInstancePosition(JobCEC(0, 0)),
              testing::Le(jobOrder.GetJobStartInstancePosition(JobCEC(2, 2))));
}
// TEST_F(RTDATest12, job_order_of_wrong_format) {
//   double objOld = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
//   jobOrder.RemoveInstance(JobCEC(1, 4), 13);
//   jobOrder.RemoveInstance(JobCEC(0, 0), 13);
//   jobOrder.RemoveInstance(JobCEC(2, 2), 13);

//   jobOrder.InsertStart(JobCEC(2, 2), 13);
//   jobOrder.InsertFinish(JobCEC(1, 4), 14);
//   jobOrder.InsertStart(JobCEC(0, 0), 15);
//   VectorDynamic stv =
//       LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
//   double obj = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
//   EXPECT_EQ(objOld, obj);
//   EXPECT_TRUE(gtsam::assert_equal(startTimeVector, stv));
// }

TEST_F(RTDATest12, job_order_strict_constraint)
{
  jobOrder.RemoveInstance(JobCEC(1, 4), 13);
  jobOrder.RemoveInstance(JobCEC(0, 0), 13);
  jobOrder.RemoveInstance(JobCEC(2, 2), 13);
  jobOrder.InsertStart(JobCEC(2, 2), 13);
  jobOrder.InsertFinish(JobCEC(1, 4), 14);
  jobOrder.InsertStart(JobCEC(0, 0), 15);
  VectorDynamic stv =
      LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec, "ReactionTimeObj");
  std::cout << "\n";
  jobOrder.print();
  PrintSchedule(tasksInfo, stv);
  EXPECT_THAT(GetFinishTime(JobCEC(1, 4), stv, tasksInfo) - GetStartTime(JobCEC(2, 2), stv, tasksInfo),
              testing::Ge(GlobalVariablesDAGOpt::LPTolerance * 0.9));
}

TEST(JobOrderConstructor, v_x)
{
  ScheduleOptions scheduleOptions;
  scheduleOptions.considerSensorFusion_ = 0;
  scheduleOptions.freshTol_ = 1e6;
  scheduleOptions.sensorFusionTolerance_ = 1e6;
  scheduleOptions.weightInMpRTDA_ = 0.5;
  scheduleOptions.weightInMpSf_ = 0.5;
  scheduleOptions.weightPunish_ = 1000;
  OrderOptDAG_SPACE::DAG_Model dagTasks =
      ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n4_v3.csv", "orig");
  TaskSet tasks = dagTasks.tasks;
  TaskSetInfoDerived tasksInfo(tasks);
  int processorNum = 2;
  VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
  initial << 383.249, 137.25, 2197, 4000, 6045.75, 8091.5, 1986, 3188.25, 4232, 6277.75, 8323.5, 186.249,
      1789, 2000, 3202.25, 4035, 5803, 6080.75 + 1e-5, 7803, 8126.5, 9803;
  PrintSchedule(tasksInfo, initial);
  SFOrder jobOrder(tasksInfo, initial);
  jobOrder.print();
  // EXPECT_THAT(GetFinishTime(JobCEC(3, 6), initial, tasksInfo),
  //             testing::Le(GetStartTime(JobCEC(2, 3), initial, tasksInfo)));
  EXPECT_THAT(jobOrder.GetJobFinishInstancePosition(JobCEC(3, 6)),
              testing::Le(jobOrder.GetJobStartInstancePosition(JobCEC(2, 3))));

  // VectorDynamic initialActual = SFOrderScheduling(dagTasks.tasks, tasksInfo, processorNum, sfOrder);
  // PrintSchedule(tasksInfo, initialActual);
  // gtsam::assert_equal(initial, initialActual);
}

// TEST_F(RTDATest12, why_wrong_initial) {
//   VectorDynamic initialLS = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_);
//   EXPECT_FALSE(initialLS == startTimeVector);

//   double objOld = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
//   VectorDynamic schedule1 =
//       LPOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
//   double obj = RTDAExperimentObj::TrueObj(dagTasks, tasksInfo, schedule1, scheduleOptions);
//   EXPECT_EQ(objOld, obj);
//   EXPECT_TRUE(gtsam::assert_equal(startTimeVector, schedule1));
// }
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}