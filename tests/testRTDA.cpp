
#include "sources/Factors/RTDA_Factor.h"
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

// sourceJob is not used because it is equivalent as jobChain[0]
inline double GetReactionTime(const std::vector<JobCEC> &jobChain, const VectorDynamic &startTimeVector,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  if (jobChain.size() <= 1) {
    CoutError("Cannot analyze reaction time because jobChain is too short!");
  };
  return GetFinishTime(jobChain.back(), startTimeVector, tasksInfo) -
         GetStartTime(jobChain[0], startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>>
GetMaxReactionTimeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                         const VectorDynamic &startTimeVector,
                         const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::vector<std::vector<JobCEC>> longestChains;
  longestChains.reserve(10); // there should be no more than 10 longest chains
  double maxLength = -2;
  for (const auto &pair : react_chain_map) {
    // const JobCEC &jobCurr = pair.first;
    const std::vector<JobCEC> &jobChain = pair.second;
    double lengthChainCurr = GetReactionTime(jobChain, startTimeVector, tasksInfo);
    if (lengthChainCurr > maxLength) {
      longestChains.clear();
      longestChains.push_back(jobChain);
      maxLength = lengthChainCurr;
    } else if (lengthChainCurr == maxLength) {
      longestChains.push_back(jobChain);
    }
  }
  return longestChains;
}

double GetDataAge(const std::vector<JobCEC> &jobChain, const std::vector<JobCEC> &prevJobChain,
                  const VectorDynamic &startTimeVector,
                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  if (jobChain.size() <= 1) {
    CoutError("Cannot analyze reaction time because jobChain is too short!");
  };

  JobCEC sinkJob = jobChain.back();
  sinkJob.jobId--;
  JobCEC sourceJob = jobChain[0];
  sourceJob.jobId--;
  if (sinkJob.jobId < 0 || sourceJob.jobId < 0 || prevJobChain.back() == jobChain.back())
    return -1;
  return GetFinishTime(sinkJob, startTimeVector, tasksInfo) -
         GetStartTime(sourceJob, startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>>
GetMaxDataAgeChains(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
                    const VectorDynamic &startTimeVector,
                    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
  std::vector<std::vector<JobCEC>> longestChains;
  longestChains.reserve(10); // there should be no more than 10 longest chains
  double maxLength = -2;
  for (const auto &pair : react_chain_map) {
    const JobCEC &jobCurr = pair.first;
    const std::vector<JobCEC> &jobChain = pair.second;

    JobCEC jobPrev(jobCurr.taskId, jobCurr.jobId - 1);
    if (react_chain_map.find(jobPrev) == react_chain_map.end())
      continue;
    const std::vector<JobCEC> &prevJobChain = react_chain_map.at(jobPrev);
    double lengthChainCurr = GetDataAge(jobChain, prevJobChain, startTimeVector, tasksInfo);
    if (lengthChainCurr > maxLength) {
      longestChains.clear();
      longestChains.push_back(jobChain);
      maxLength = lengthChainCurr;
    } else if (lengthChainCurr == maxLength) {
      longestChains.push_back(jobChain);
    }
  }
  return longestChains;
}

class LongestCAChain {
public:
  LongestCAChain() {}
  LongestCAChain(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                 const VectorDynamic &startTimeVector, int processorNum)
      : longestChains_(FindLongestCAChain(dagTasks, tasksInfo, jobOrder, startTimeVector, processorNum)) {}

  std::vector<std::vector<JobCEC>> FindLongestCAChain(const DAG_Model &dagTasks,
                                                      const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                                                      const VectorDynamic &startTimeVector,
                                                      int processorNum) {
    std::vector<std::vector<JobCEC>> longestChains;
    longestChains.reserve(dagTasks.chains_.size() * 3); // 3x is usually not necessary, though

    for (uint i = 0; i < dagTasks.chains_.size(); i++) {
      auto react_chain_map =
          GetReactionChainMap(dagTasks, tasksInfo, jobOrder, processorNum, dagTasks.chains_[i], i);
      auto chains = GetMaxReactionTimeChains(react_chain_map, startTimeVector, tasksInfo);
      longestChains.insert(longestChains.end(), chains.begin(), chains.end());
      chains = GetMaxDataAgeChains(react_chain_map, startTimeVector, tasksInfo);
      longestChains.insert(longestChains.end(), chains.begin(), chains.end());
    }
    return longestChains;
  }

  void FindLongestCAChain() {}

  // even index for reaction time chains, odd index for data age chains
  std::vector<JobCEC> GetReactionChain(int chainIndex);

  // data members
  std::vector<std::vector<JobCEC>> longestChains_;
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
  // std::vector<std::vector<JobCEC>> longestChains;
  // longestChains.reserve(3);
  std::vector<JobCEC> chain1 = {JobCEC(0, 0), JobCEC(2, 1)}; // RT
  std::vector<JobCEC> chain2 = {JobCEC(0, 1), JobCEC(2, 2)}; // RT & DA
  std::vector<JobCEC> chain3 = {JobCEC(0, 2), JobCEC(2, 1)}; // RT

  AssertEqualVectorNoRepeat<JobCEC>(chain1, longestChain.longestChains_[0], 0, 143);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}