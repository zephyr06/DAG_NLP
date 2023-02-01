
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
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
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};

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

class RTDATest3 : public RTDATest2 {
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
    jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  }
};

TEST_F(RTDATest1, ExtractIndependentJobGroups) {
  auto jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  EXPECT_EQ(0, jobGroupMap[JobCEC(0, 0)]);
  EXPECT_EQ(0, jobGroupMap[JobCEC(0, 1)]);
  EXPECT_EQ(0, jobGroupMap[JobCEC(1, 0)]);
  EXPECT_EQ(0, jobGroupMap[JobCEC(2, 0)]);
}

TEST_F(RTDATest2, ExtractIndependentJobGroups) {
  EXPECT_EQ(0, jobGroupMap[JobCEC(0, 0)]);
  EXPECT_EQ(0, jobGroupMap[JobCEC(0, 1)]);
  EXPECT_EQ(1, jobGroupMap[JobCEC(0, 2)]);
}

TEST_F(RTDATest1, WhetherInfluenceJobSimple) {
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(0, 1), jobGroupMap));
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(1, 0), jobGroupMap));
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(2, 0), jobGroupMap));
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(1, 0), JobCEC(0, 0), jobGroupMap));
}

TEST_F(RTDATest1, WhetherInfluenceJobSource) {
  // arg order: jobCurr, jobChanged
  EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 0), JobCEC(0, 1), jobGroupMap, jobOrder, 5, 6, tasksInfo,
                                         startTimeVector));
  EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, 6, 7, tasksInfo,
                                         startTimeVector));
  EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 1), JobCEC(0, 0), jobGroupMap, jobOrder, 2, 3, tasksInfo,
                                         startTimeVector));
  EXPECT_TRUE(WhetherInfluenceJobSource(JobCEC(1, 0), JobCEC(2, 0), jobGroupMap, jobOrder, 0, 1, tasksInfo,
                                        startTimeVector));
}
TEST_F(RTDATest1, WhetherInfluenceJobSink) {
  // arg order: jobCurr, jobChanged
  EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(0, 1), jobGroupMap, jobOrder, 5, 6, tasksInfo,
                                       startTimeVector));
  EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, 6, 7, tasksInfo,
                                       startTimeVector));
  EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 1), JobCEC(0, 0), jobGroupMap, jobOrder, 2, 3, tasksInfo,
                                       startTimeVector));
  EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(1, 0), JobCEC(2, 0), jobGroupMap, jobOrder, 0, 1, tasksInfo,
                                       startTimeVector));
  EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(1, 0), JobCEC(2, 0), jobGroupMap, jobOrder, 6, 7, tasksInfo,
                                      startTimeVector));
  EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(2, 0), jobGroupMap, jobOrder, 2, 3, tasksInfo,
                                      startTimeVector));
}

TEST_F(RTDATest2, WhetherInfluenceJobSimple) {
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(0, 1), jobGroupMap));
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(1, 1), jobGroupMap));
  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(2, 0), jobGroupMap));
  EXPECT_FALSE(WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(1, 2), jobGroupMap));

  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(2, 0), JobCEC(0, 0), jobGroupMap));
}
TEST_F(RTDATest3, WhetherInfluenceJobSimple) {

  EXPECT_TRUE(WhetherInfluenceJobSimple(JobCEC(2, 0), JobCEC(1, 1), jobGroupMap));
}

class JobPositionTest : public ::testing::Test {
protected:
  void SetUp() override { jobPosition = JobPosition(10, 15); }
  JobPosition jobPosition;
};
TEST_F(JobPositionTest, UpdateAfterRemoveInstance) {
  jobPosition.UpdateAfterRemoveInstance(5);
  EXPECT_EQ(9, jobPosition.start_);
  EXPECT_EQ(14, jobPosition.finish_);
  jobPosition.UpdateAfterRemoveInstance(20);
  EXPECT_EQ(9, jobPosition.start_);
  EXPECT_EQ(14, jobPosition.finish_);

  jobPosition.UpdateAfterInsertInstance(5);
  EXPECT_EQ(10, jobPosition.start_);
  EXPECT_EQ(15, jobPosition.finish_);

  jobPosition.UpdateAfterInsertInstance(20);
  EXPECT_EQ(10, jobPosition.start_);
  EXPECT_EQ(15, jobPosition.finish_);

  jobPosition.UpdateAfterRemoveInstance(12);
  EXPECT_EQ(10, jobPosition.start_);
  EXPECT_EQ(14, jobPosition.finish_);

  jobPosition.UpdateAfterInsertInstance(12);
  EXPECT_EQ(10, jobPosition.start_);
  EXPECT_EQ(15, jobPosition.finish_);
}

class RTDATest4 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v40";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(13);
    startTimeVector << 123, 0, 1000, 0, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
  }
};

TEST_F(RTDATest4, whether_break_chain) {
  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);
  std::unordered_map<JobCEC, int> jobGroupMap_ = ExtractIndependentJobGroups(jobOrder, tasksInfo);
  JobCEC jobRelocate(2, 0);
  LLint startP = 3;
  LLint finishP = 4;
  EXPECT_FALSE(
      WhetherJobBreakChain(jobRelocate, startP, finishP, longestChain, dagTasks, jobOrder, tasksInfo));
  JobCEC sinkJob = longestChain[0].back();
  EXPECT_TRUE(WhetherInfluenceJobSink(sinkJob, jobRelocate, jobGroupMap_, jobOrder, startP, finishP,
                                      tasksInfo, startTimeVector));
}

class RTDATest5 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v41";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(17);
    startTimeVector << 0, 1205, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 678, 5000, 0, 3173, 4000,
        6000, 8000;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
  }
};
// TEST_F(RTDATest5, FindLongestCAChain) {
//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_); EXPECT_EQ(2, longestChain.size());

//   std::vector<JobCEC> chain0 = {JobCEC(2, 3), JobCEC(1, 2)}; // RT
//   std::vector<JobCEC> chain2 = {JobCEC(2, 2), JobCEC(1, 1)}; // RT&DA
//   AssertEqualVectorNoRepeat<JobCEC>(chain0, longestChain.longestChains_[0], 0, __LINE__);
//   AssertEqualVectorNoRepeat<JobCEC>(chain2, longestChain.longestChains_[1], 0, __LINE__);
// }

class RTDATest6 : public RTDATest1 {
  void SetUp() override {
    std::string taskSetName = "test_n3_v42";
    SetUpTaskSet(taskSetName);
    startTimeVector = GenerateVectorDynamic(5);
    startTimeVector << 712, 1176, 0, 0, 1000;
    jobOrder = SFOrder(tasksInfo, startTimeVector);
    jobOrder.print();
  }
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

// previous adjacent job means the jobs whose finish time equals the start time of jobCurr
std::vector<JobCEC> FindPrevAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                        const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                        const VectorDynamic &startTimeVector) {
  // LLint jobCurrOldStart = jobOrder.GetJobStartInstancePosition(job);
  double startTime = GetStartTime(job, startTimeVector, tasksInfo);
  std::vector<JobCEC> prevAdjacentJobs;
  prevAdjacentJobs.reserve(4 * 2); // actually, cannot be more than #core*2

  std::unordered_set<JobCEC> record;
  record.reserve(4 * 2);
  auto AddAdjacentJob = [&](LLint index) {
    for (int i = index - 1; i >= 0; i--) {
      TimeInstance instCurr = jobOrder[i];
      if (instCurr.type == 'f') {
        JobCEC jobCurr = instCurr.job;
        double jobCurrfinishTime = GetFinishTime(jobCurr, startTimeVector, tasksInfo);
        if (std::abs(jobCurrfinishTime - startTime) < 1e-3) {
          if (record.find(jobCurr) == record.end()) {
            prevAdjacentJobs.push_back(jobCurr);
            record.insert(jobCurr);
          }
        } else {
          break;
        }
      }
    }
  };

  AddAdjacentJob(jobOrder.GetJobFinishInstancePosition(job));
  AddAdjacentJob(jobOrder.GetJobStartInstancePosition(job));
  return prevAdjacentJobs;
}
TEST_F(RTDATest7, FindPrevAdjacentJob) {
  auto prevAdjacentJobs = FindPrevAdjacentJob(JobCEC(0, 0), jobOrder, tasksInfo, startTimeVector);
  EXPECT_EQ(1, prevAdjacentJobs.size());
  EXPECT_TRUE(JobCEC(2, 0) == prevAdjacentJobs[0]);
}

// we need to exam all the jobs that are closely adjacent to jobCurr;
bool WhetherInfluenceJobAndAfterSink(JobCEC jobCurr, const JobCEC &jobChanged,
                                     std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                     LLint startP, LLint finishP,
                                     const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                     const VectorDynamic &startTimeVector) {
  // Exam whether the jobs that are closely adjacent to jobCurr will be influenced by jobChanged
  if (WhetherInfluenceJobSource(jobCurr, jobChanged, jobGroupMap, jobOrder, startP, finishP, tasksInfo,
                                startTimeVector))
    return true;

  // Termination conditions
  double jobCurrStartTime = GetStartTime(jobCurr, startTimeVector, tasksInfo);
  if (jobCurrStartTime == GetActivationTime(jobCurr, tasksInfo))
    return false;

  std::vector<JobCEC> prevAdjacentJob = FindPrevAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
  for (auto job : prevAdjacentJob) {
    if (WhetherInfluenceJobAndAfterSink(job, jobChanged, jobGroupMap, jobOrder, startP, finishP, tasksInfo,
                                        startTimeVector))
      return true;
  }

  return false;
}

TEST_F(RTDATest7, WhetherInfluenceJobAndAfterSink) {
  LLint startP = 0;
  LLint finishP = 2;
  EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP, finishP,
                                      tasksInfo, startTimeVector));

  EXPECT_TRUE(WhetherInfluenceJobAndAfterSink(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP,
                                              finishP, tasksInfo, startTimeVector));
  EXPECT_TRUE(WhetherInfluenceJobAndAfterSink(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP,
                                              finishP, tasksInfo, startTimeVector));
}
// TEST_F(RTDATest6, FindLongestCAChain) {
//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_); EXPECT_EQ(2, longestChain.size());

//   std::vector<JobCEC> chain0 = {JobCEC(1, 0), JobCEC(0, 1)}; // RT
//   std::vector<JobCEC> chain2 = {JobCEC(1, 0), JobCEC(0, 2)}; // RT&DA
//   AssertEqualVectorNoRepeat<JobCEC>(chain0, longestChain.longestChains_[0], 0, __LINE__);
//   AssertEqualVectorNoRepeat<JobCEC>(chain2, longestChain.longestChains_[1], 0, __LINE__);
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}