

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
// returns whether instCompare is an immediate forward adjacent job to instCurr
bool WhetherImmediateForwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                     const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                     const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                     double tolerance = 1e-3) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) <
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  }

  return false;
}

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

// previous adjacent job means the jobs whose finish time equals the start time of jobCurr
std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                           const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                           const VectorDynamic &startTimeVector) {
  std::vector<JobCEC> prevAdjacentJobs;
  prevAdjacentJobs.reserve(4 * 2); // actually, cannot be more than #core*2

  std::unordered_set<JobCEC> record;
  record.reserve(4 * 2);

  LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
  TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
  auto AddImmediateAdjacentInstance = [&](TimeInstance &instCurrJob, LLint jobInstIndex) {
    for (int i = jobInstIndex - 1; i >= 0; i--) {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateForwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder)) {
        if (record.find(instIte.job) == record.end()) {
          prevAdjacentJobs.push_back(instIte.job);
          record.insert(instIte.job);
        } else
          break;
      } else
        break;
    }
  };
  AddImmediateAdjacentInstance(instCurrJobStart, jobStartIndex);

  LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
  TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
  AddImmediateAdjacentInstance(instCurrJobFinish, jobFinishIndex);
  return prevAdjacentJobs;
}

// TODO: consider utilize startP and finishP
// Assumption: start time of jobCurr in startTimeVector cannot move earlier
bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
                                  std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector) {
  // ,
  // std::unordered_set<JobCEC> &pathRecord, int &countPath
  // if (countPath > 10)
  //   return true;
  if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) - GetActivationTime(jobCurr, tasksInfo)) <
      1e-3)
    return false;
  else if (jobCurr == jobChanged) {
    // countPath++;
    return true;
  }

  std::vector<JobCEC> prevAdjacentJobs =
      FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (prevAdjacentJobs.size() == 0)
    return false; // consider more, should be false under assumptions  of startTimeVector

  for (auto &jobPrev : prevAdjacentJobs) {
    if (WhetherJobStartEarlierHelper(jobPrev, jobChanged, jobGroupMap, jobOrder, tasksInfo,
                                     startTimeVector) == false)
      return false;
  }

  return true; // all the conditions known to return false failed
}

bool WhetherJobStartEarlier(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                            SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector) {
  jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
  jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

  // Current analysis on job group is not safe, let's' see what we can do without it
  // if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
  //   return false;
  return WhetherJobStartEarlierHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector);
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

bool WhetherImmediateBackwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                      const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                      const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                      double tolerance = 1e-3) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) >
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) >
              jobOrder.GetJobStartInstancePosition(instCurr.job)) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobStartInstancePosition(instCompare.job) >
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance &&
          jobOrder.GetJobFinishInstancePosition(instCompare.job) >
              jobOrder.GetJobFinishInstancePosition(instCurr.job)) {
        return true;
      }
    }
  }

  return false;
}

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

// 'backward' means finding the jobs whose index is larger than job, i.e., -->
std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                            const VectorDynamic &startTimeVector) {
  std::vector<JobCEC> followAdjacentJobs;
  followAdjacentJobs.reserve(4 * 2); // actually, cannot be more than #core*2

  std::unordered_set<JobCEC> record;
  record.reserve(4 * 2);

  LLint jobStartIndex = jobOrder.GetJobStartInstancePosition(job);
  TimeInstance instCurrJobStart = jobOrder[jobStartIndex];
  auto AddImmediateAdjacentInstance = [&](TimeInstance &instCurrJob, LLint jobInstIndex) {
    for (uint i = jobInstIndex + 1; i < jobOrder.size(); i++) {
      TimeInstance instIte = jobOrder[i];
      if (WhetherImmediateBackwardAdjacent(instCurrJob, instIte, tasksInfo, startTimeVector, jobOrder)) {
        if (record.find(instIte.job) == record.end()) {
          followAdjacentJobs.push_back(instIte.job);
          record.insert(instIte.job);
        } else
          break;
      } else
        break;
    }
  };
  AddImmediateAdjacentInstance(instCurrJobStart, jobStartIndex);

  LLint jobFinishIndex = jobOrder.GetJobFinishInstancePosition(job);
  TimeInstance instCurrJobFinish = jobOrder[jobFinishIndex];
  AddImmediateAdjacentInstance(instCurrJobFinish, jobFinishIndex);
  return followAdjacentJobs;
}

bool WhetherJobStartLaterHelper(JobCEC jobCurr, JobCEC jobChanged,
                                std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                const VectorDynamic &startTimeVector) {
  if (std::abs(GetFinishTime(jobCurr, startTimeVector, tasksInfo) - GetDeadline(jobCurr, tasksInfo)) < 1e-3)
    return false;
  else if (jobCurr == jobChanged) {
    return true;
  }

  std::vector<JobCEC> followAdjacentJobs =
      FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  if (followAdjacentJobs.size() == 0) // this should never happen in case of LP order scheduler, and this
                                      // function should not be used with simple order scheduler
    return false;

  for (auto &jobFollow : followAdjacentJobs) {
    if (WhetherJobStartLaterHelper(jobFollow, jobChanged, jobGroupMap, jobOrder, tasksInfo,
                                   startTimeVector) == false)
      return false;
  }

  return true;
}

// this function requires that the start time is already maximum in startTimeVector
// this function cannot work with SimpleOrderScheduler
bool WhetherJobStartLater(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                          SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                          const VectorDynamic &startTimeVector) {
  jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
  jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

  // Current analysis on job group is not safe, let's' see what we can do without it
  // if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
  //   return false;
  // int countPath = 0;
  // std::unordered_set<JobCEC> pathRecord;
  // pathRecord.reserve(tasksInfo.length);

  return WhetherJobStartLaterHelper(jobCurr, jobChanged, jobGroupMap, jobOrder, tasksInfo, startTimeVector);
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}