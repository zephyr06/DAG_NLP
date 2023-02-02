

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"
#include <functional>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}