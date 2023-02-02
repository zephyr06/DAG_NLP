

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

bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector, double tolerance = 1e-3) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    }
  }

  return false;
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
// bool WhetherInfluenceJobAndAfterSink(JobCEC jobCurr, const JobCEC &jobChanged,
//                                      std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
//                                      LLint startP, LLint finishP,
//                                      const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                                      const VectorDynamic &startTimeVector) {
//   // Exam whether the jobs that are closely adjacent to jobCurr will be influenced by jobChanged
//   if (WhetherInfluenceJobSource(jobCurr, jobChanged, jobGroupMap, jobOrder, startP, finishP, tasksInfo,
//                                 startTimeVector))
//     return true;

//   // Termination conditions
//   double jobCurrStartTime = GetStartTime(jobCurr, startTimeVector, tasksInfo);
//   if (jobCurrStartTime == GetActivationTime(jobCurr, tasksInfo))
//     return false;

//   std::vector<JobCEC> prevAdjacentJob = FindPrevAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
//   for (auto job : prevAdjacentJob) {
//     if (WhetherInfluenceJobAndAfterSink(job, jobChanged, jobGroupMap, jobOrder, startP, finishP, tasksInfo,
//                                         startTimeVector))
//       return true;
//   }

//   return false;
// }

// TEST_F(RTDATest7, WhetherInfluenceJobAndAfterSink) {
//   LLint startP = 0;
//   LLint finishP = 2;
//   EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP, finishP,
//                                       tasksInfo, startTimeVector));

//   EXPECT_TRUE(WhetherInfluenceJobAndAfterSink(JobCEC(2, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP,
//                                               finishP, tasksInfo, startTimeVector));
//   EXPECT_TRUE(WhetherInfluenceJobAndAfterSink(JobCEC(0, 0), JobCEC(1, 0), jobGroupMap, jobOrder, startP,
//                                               finishP, tasksInfo, startTimeVector));
// }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}