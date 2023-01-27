
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

int FindSiblingJobIndex(const JobCEC &job, const std::vector<JobCEC> &jobChainCurr) {
  for (uint i = 0; i < jobChainCurr.size(); i++) {
    if (job.taskId == jobChainCurr[i].taskId) {
      return i;
    }
  }
  return -1;
}
// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove job, and then insert its start/finish instances at
// startP/finishP
// If you want to be safer, return more 'true'
bool WhetherJobBreakChain(const JobCEC &job, LLint startP, LLint finishP,
                          const LongestCAChain &longestJobChains, const DAG_Model &dagTasks,
                          SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo) {

  for (auto &taskChainCurr : dagTasks.chains_) {
    auto itr = std::find(taskChainCurr.begin(), taskChainCurr.end(), job.taskId);
    if (itr != taskChainCurr.end()) {
      for (uint i = 0; i < longestJobChains.size(); i++) {
        const std::vector<JobCEC> &jobChainCurr = longestJobChains[i];
        int siblingJobIndex = FindSiblingJobIndex(job, jobChainCurr);
        JobCEC sibJob = jobChainCurr[siblingJobIndex];
        if (sibJob == job)
          return true; // this may not be necessary, but is a safe solution

        if (siblingJobIndex == 0) { // new source job may initiate a different cause-effect chain
          JobCEC afterSibJob =
              jobChainCurr[siblingJobIndex + 1]; // assume the length of the chain is longer than 1
          if (startP > jobOrder.GetJobFinishInstancePosition(sibJob) &&
              finishP < jobOrder.GetJobStartInstancePosition(afterSibJob))
            return true;
        } else { // whether the job reacts earlier than sibingJob?
          if (sibJob.jobId < job.jobId)
            continue; // the job cannot react earlier than sibJob, and so cannot change reaction relationship
          JobCEC sibJobImmediateSourceJob = jobChainCurr[siblingJobIndex - 1];
          LLint sibImmeSourJobFinish = jobOrder.GetJobFinishInstancePosition(sibJobImmediateSourceJob);
          if (jobOrder.GetJobStartInstancePosition(job) < sibImmeSourJobFinish)
            sibImmeSourJobFinish--;
          if (jobOrder.GetJobFinishInstancePosition(job) < sibImmeSourJobFinish)
            sibImmeSourJobFinish--;
          if (sibImmeSourJobFinish <= startP && startP < jobOrder.GetJobStartInstancePosition(sibJob))
            return true;
        }
      }
    }
  }

  return false;
}

TEST_F(RTDATest1, break_chain) {

  LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector, scheduleOptions.processorNum_);

  EXPECT_TRUE(WhetherJobBreakChain(job0, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job0, 0, 1, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_TRUE(WhetherJobBreakChain(job2, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
  EXPECT_FALSE(WhetherJobBreakChain(job1, 0, 0, longestChain, dagTasks, jobOrder, tasksInfo));
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}