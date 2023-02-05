

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/JobStartAnalysis.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"
#include <functional>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

class RTDATest1 : public ::testing::Test {
protected:
  void SetUp() override {
    std::string taskSetName = "test_n3_v18";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(4);
    startTimeVector << 0, 10, 1, 0;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
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
  std::unordered_map<JobCEC, int> jobGroupMap;

  JobCEC job0 = JobCEC(0, 0);
  JobCEC job1 = JobCEC(0, 1);
  JobCEC job2 = JobCEC(0, 2);
  JobCEC job3 = JobCEC(0, 3);
};

class RTDATest7 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 4592, 0, 2000, 4680, 6000, 8000, 782, 6000;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest7, WhetherImmediateForwardAdjacent_v1) {
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

TEST_F(RTDATest7, WhetherImmediateAdjacent_v1) {
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

TEST_F(RTDATest7, FindForwardAdjacentJob) {
  auto prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(2, 0) == prevAdjacentJobs[0]);

  prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(2, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 0) == prevAdjacentJobs[0]);

  startTimeVector(5) = 9028;
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  jobOrder.print();
  prevAdjacentJobs = FindForwardAdjacentJob(JobCEC(2, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(2, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 3) == prevAdjacentJobs[0]);
  EXPECT_TRUE(JobCEC(1, 4) == prevAdjacentJobs[1]);
}

TEST_F(RTDATest7, WhetherJobStartEarlier_v1) {
  // LLint startP = 0;
  // LLint finishP = 2;

  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));

  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest7, WhetherJobStartEarlier_v2) {
  EXPECT_FALSE(
      WhetherJobStartEarlier(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}
// *****************************************************************************************************************

TEST_F(RTDATest7, WhetherImmediateBackwardAdjacent_v1) {
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

class RTDATest8 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 5320, 0, 2000, 4626, 6000, 9218, 782, 5408;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};

TEST_F(RTDATest8, FindBackwardAdjacentJob) {

  auto prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(2, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 4) == prevAdjacentJobs[0]);

  prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(2, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(1, 2) == prevAdjacentJobs[0]);
  EXPECT_TRUE(JobCEC(2, 1) == prevAdjacentJobs[1]);
}

TEST_F(RTDATest8, WhetherJobStartLater) {
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
TEST_F(RTDATest8, WhetherJobStartLater_v2) {
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(0, 0), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(1, 2), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 4), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_TRUE(
      WhetherJobStartLater(JobCEC(1, 0), JobCEC(2, 0), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest8, WhetherJobStartLater_v3) {
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  startTimeVector << 3810, 0, 2000, 4000, 6000, 8000, 0, 6000;
  // this function doesn't work with SimpleOrderScheduler
  jobOrder = SFOrder(tasksInfo, startTimeVector);
  // EXPECT_TRUE(
  //     WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

class RTDATest9 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v43";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(8);
    startTimeVector << 3810, 0, 2000, 4000, 7218, 9218, 0, 6190;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};
TEST_F(RTDATest9, WhetherJobStartLater_v4) {
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 1), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
  EXPECT_FALSE(
      WhetherJobStartLater(JobCEC(2, 0), JobCEC(1, 1), jobGroupMap, jobOrder, tasksInfo, startTimeVector));
}

TEST_F(RTDATest9, FindBackwardAdjacentJob) {
  auto prevAdjacentJobs = FindBackwardAdjacentJob(JobCEC(1, 1), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(0, prevAdjacentJobs.size());
}

TEST_F(RTDATest8, LP_Optimality) {
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

struct CentralJobs {
  CentralJobs(size_t size) {
    forwardJobs.reserve(size);
    backwardJobs.reserve(size);
  }
  CentralJobs(std::vector<JobCEC> &forwardJobs, std::vector<JobCEC> &backwardJobs)
      : forwardJobs(forwardJobs), backwardJobs(backwardJobs) {}

  std::vector<JobCEC> forwardJobs;
  std::vector<JobCEC> backwardJobs;
};

CentralJobs FindCentralJob(const LongestCAChain &longestChain,
                           const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::unordered_set<JobCEC> centralSourceJobRecord;
  centralSourceJobRecord.reserve(tasksInfo.length);
  std::unordered_set<JobCEC> centralSinkJobRecord;
  centralSinkJobRecord.reserve(tasksInfo.length);
  CentralJobs centralJobs(tasksInfo.length);

  for (uint i = 0; i < longestChain.size(); i++) {
    auto &chainCurr = longestChain.longestChains_[i];
    if (chainCurr.size() > 0) {
      JobCEC jobSource = chainCurr[0].GetJobWithinHyperPeriod(tasksInfo);
      if (centralSourceJobRecord.find(jobSource) == centralSourceJobRecord.end()) {
        centralSourceJobRecord.insert(jobSource);
        centralJobs.backwardJobs.push_back(jobSource);
      }

      JobCEC jobSink = ((chainCurr.back())).GetJobWithinHyperPeriod(tasksInfo);
      if (centralSinkJobRecord.find(jobSink) == centralSinkJobRecord.end()) {
        centralSinkJobRecord.insert(jobSink);
        centralJobs.forwardJobs.push_back(jobSink);
      }
    }
  }
  return centralJobs;
}

TEST_F(RTDATest8, FindCentralForwardJob) {
  std::vector<JobCEC> centralSourceJob = {JobCEC(2, 0), JobCEC(2, 1)};
  std::vector<JobCEC> centralSinkJob = {JobCEC(0, 0)};
  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);
  auto centralJobActual = FindCentralJob(longestChain, tasksInfo);
  AssertEqualVectorNoRepeat<JobCEC>(centralSourceJob, centralJobActual.backwardJobs, 1e-3, __LINE__);
  AssertEqualVectorNoRepeat<JobCEC>(centralSinkJob, centralJobActual.forwardJobs, 1e-3, __LINE__);
}

std::vector<JobCEC> FindActiveJob(const CentralJobs &centralJob,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::vector<JobCEC> activeJobs;
  return activeJobs;
}

TEST_F(RTDATest8, FindActiveJob) {
  std::vector<JobCEC> activeJob = {JobCEC(0, 0), JobCEC(2, 0), JobCEC(2, 1), JobCEC(1, 4)};
  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);
  CentralJobs centralJob = FindCentralJob(longestChain, tasksInfo);

  std::vector<JobCEC> activeJobActual = FindActiveJob(centralJob, tasksInfo);
  // AssertEqualVectorNoRepeat<JobCEC>(activeJob, activeJobActual, 1e-3, __LINE__);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}