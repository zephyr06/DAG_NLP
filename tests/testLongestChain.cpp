
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <functional>

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"

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
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +
                                     "TaskData/" + taskSet + ".csv",
                                 "orig");
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);

        scheduleOptions.freshTol_ = 1e6;
        scheduleOptions.sensorFusionTolerance_ = 1e6;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 1000;

        scheduleOptions.selectInitialFromPoolCandidate_ =
            0;  // 1000 if adding select initial from pool
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
    auto react_chain_map = GetReactionChainMap(dagTasks, tasksInfo, jobOrder,
                                               scheduleOptions.processorNum_,
                                               dagTasks.chains_[0], 0);
    auto chain0 = react_chain_map.at(job0);
    EXPECT_EQ(23, GetReactionTime(chain0, startTimeVector, tasksInfo));
    // EXPECT_EQ(-1, GetDataAge(chain0, startTimeVector, tasksInfo));
    auto chain1 = react_chain_map.at(job1);
    EXPECT_EQ(13, GetReactionTime(chain1, startTimeVector, tasksInfo));
    auto chain2 = react_chain_map.at(job2);
    EXPECT_EQ(23, GetReactionTime(chain2, startTimeVector, tasksInfo));
    auto chain3 = react_chain_map.at(job3);
    EXPECT_EQ(13, GetReactionTime(chain3, startTimeVector, tasksInfo));
}
TEST_F(RTDATest1, GetDataAge) {
    auto da_chain_map = GetDataAgeChainMap(dagTasks, tasksInfo, jobOrder,
                                           scheduleOptions.processorNum_,
                                           dagTasks.chains_[0], 0);

    EXPECT_EQ(
        13, GetDataAge(da_chain_map[JobCEC(2, 0)], startTimeVector, tasksInfo));
}

TEST_F(RTDATest1, break_chain_rt) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_,
                                "ReactionTimeObj");

    EXPECT_TRUE(WhetherJobBreakChainRT(job0, 0, 0, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_TRUE(WhetherJobBreakChainRT(job0, 0, 1, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainRT(job1, 6, 7, longestChain, dagTasks,
                                        jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainRT(JobCEC(1, 0), 0, 0, longestChain,
                                        dagTasks, jobOrder, tasksInfo));
}
TEST_F(RTDATest1, break_chain_da) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_, "DataAgeObj");
    // Longest job chain: J_{0, -1} -> J_{2, 0}
    EXPECT_TRUE(WhetherJobBreakChainDA(job0, 0, 0, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_TRUE(WhetherJobBreakChainDA(job0, 0, 1, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_TRUE(WhetherJobBreakChainDA(job1, 0, 0, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainDA(JobCEC(1, 0), 0, 0, longestChain,
                                        dagTasks, jobOrder, tasksInfo));
}
TEST_F(RTDATest1, ExtractIndependentJobGroups) {
    auto jobGroupMap = ExtractIndependentJobGroups(jobOrder, tasksInfo);
    EXPECT_EQ(0, jobGroupMap[JobCEC(0, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(0, 1)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(1, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(2, 0)]);
}

TEST_F(RTDATest1, WhetherInfluenceJobSimple) {
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(0, 1), jobGroupMap));
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(1, 0), jobGroupMap));
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 1), JobCEC(2, 0), jobGroupMap));
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(1, 0), JobCEC(0, 0), jobGroupMap));
}

class ObjExperimentObjTest_n5_v2 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v2.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 1, 2};
        dagTasks.chains_[0] = chain1;

        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
};
TEST_F(ObjExperimentObjTest_n5_v2, GetDataAgeChainMap) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(6);
    initialEstimate << 1, 100, 2, 3, 4, 5;
    SFOrder jobOrder(tasksInfo, initialEstimate);
    jobOrder.print();
    auto react_chain_map = GetDataAgeChainMap(dagTasks, tasksInfo, jobOrder,
                                              scheduleOptions.processorNum_,
                                              dagTasks.chains_[0], 0);
    auto chain_actual = react_chain_map.at(JobCEC(2, 0));
    std::vector<JobCEC> chain_expected = {JobCEC(0, -3), JobCEC(1, -1),
                                          JobCEC(2, 0)};
    AssertEqualVectorExact<JobCEC>(chain_expected, chain_actual, 0, __LINE__);
}

class ObjExperimentObjTest_n5_v4 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v4.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 1, 2};
        dagTasks.chains_ = {chain1};
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
};
TEST_F(ObjExperimentObjTest_n5_v4, GetDataAgeChainMap) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(10);
    initialEstimate << 0, 100, 200, 300, 1, 2, 202, 3, 203, 4;
    SFOrder jobOrder(tasksInfo, initialEstimate);
    jobOrder.print();
    auto react_chain_map = GetDataAgeChainMap(dagTasks, tasksInfo, jobOrder,
                                              scheduleOptions.processorNum_,
                                              dagTasks.chains_[0], 0);
    auto chain_actual = react_chain_map.at(JobCEC(2, 0));
    std::vector<JobCEC> chain_expected = {JobCEC(0, -5), JobCEC(1, -1),
                                          JobCEC(2, 0)};
    AssertEqualVectorExact<JobCEC>(chain_expected, chain_actual, 0, __LINE__);

    chain_actual = react_chain_map.at(JobCEC(2, 1));
    chain_expected = {JobCEC(0, -1), JobCEC(1, 0), JobCEC(2, 1)};
    AssertEqualVectorExact<JobCEC>(chain_expected, chain_actual, 0, __LINE__);
}
// TODO: FIX THIS TEST
// TEST_F(RTDATest1, longest_chain) {

//   auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0],
//   startTimeVector);

//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_);
//   // order:
//   // std::vector<std::vector<JobCEC>> longestChains;
//   // longestChains.reserve(3);
//   std::vector<JobCEC> chain0 = {JobCEC(0, 0), JobCEC(2, 1)}; // RT
//   std::vector<JobCEC> chain2 = {JobCEC(0, 2), JobCEC(2, 2)}; // RT&DA
//   EXPECT_EQ(2, longestChain.longestChains_.size());
//   AssertEqualVectorNoRepeat<JobCEC>(chain2, longestChain.longestChains_[0],
//   0, __LINE__); AssertEqualVectorNoRepeat<JobCEC>(chain0,
//   longestChain.longestChains_[1], 0, __LINE__);
// }

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

TEST_F(RTDATest2, break_chain_rt) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_,
                                "ReactionTimeObj");
    JobCEC job10(1, 0);
    JobCEC job11(1, 1);
    JobCEC job12(1, 2);
    EXPECT_FALSE(WhetherJobBreakChainRT(job10, 1, 2, longestChain, dagTasks,
                                        jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainRT(job10, 0, 1, longestChain, dagTasks,
                                        jobOrder, tasksInfo));
    EXPECT_TRUE(WhetherJobBreakChainRT(job11, 1, 2, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainRT(job12, 10, 11, longestChain, dagTasks,
                                        jobOrder, tasksInfo));
}

TEST_F(RTDATest2, break_chain_da) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_, "DataAgeObj");
    JobCEC job10(1, 0);
    JobCEC job11(1, 1);
    JobCEC job12(1, 2);
    EXPECT_TRUE(WhetherJobBreakChainDA(job12, 1, 2, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_TRUE(WhetherJobBreakChainDA(job10, 1, 2, longestChain, dagTasks,
                                       jobOrder, tasksInfo));
    EXPECT_FALSE(WhetherJobBreakChainDA(job12, 10, 11, longestChain, dagTasks,
                                        jobOrder, tasksInfo));
}
TEST_F(RTDATest2, ExtractIndependentJobGroups) {
    EXPECT_EQ(0, jobGroupMap[JobCEC(0, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(0, 1)]);
    EXPECT_EQ(1, jobGroupMap[JobCEC(0, 2)]);
}
TEST_F(RTDATest2, WhetherInfluenceJobSimple) {
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(0, 1), jobGroupMap));
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(1, 1), jobGroupMap));
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(2, 0), jobGroupMap));
    EXPECT_FALSE(
        WhetherInfluenceJobSimple(JobCEC(0, 0), JobCEC(1, 2), jobGroupMap));

    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(2, 0), JobCEC(0, 0), jobGroupMap));
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
TEST_F(RTDATest7, jobGroupMap) {
    EXPECT_EQ(0, jobGroupMap[JobCEC(1, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(2, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(1, 1)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(0, 0)]);
    EXPECT_EQ(0, jobGroupMap[JobCEC(1, 2)]);
    EXPECT_EQ(1, jobGroupMap[JobCEC(1, 3)]);  // this should be 0?
    EXPECT_EQ(1, jobGroupMap[JobCEC(2, 1)]);
    EXPECT_EQ(1, jobGroupMap[JobCEC(1, 4)]);
}

// TEST_F(RTDATest1, WhetherInfluenceJobSource) {
//   // arg order: jobCurr, jobChanged
//   EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 0), JobCEC(0, 1),
//   jobGroupMap, jobOrder, 5, 6, tasksInfo,
//                                          startTimeVector));
//   EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 0), JobCEC(1, 0),
//   jobGroupMap, jobOrder, 6, 7, tasksInfo,
//                                          startTimeVector));
//   EXPECT_FALSE(WhetherInfluenceJobSource(JobCEC(0, 1), JobCEC(0, 0),
//   jobGroupMap, jobOrder, 2, 3, tasksInfo,
//                                          startTimeVector));
//   EXPECT_TRUE(WhetherInfluenceJobSource(JobCEC(1, 0), JobCEC(2, 0),
//   jobGroupMap, jobOrder, 0, 1, tasksInfo,
//                                         startTimeVector));
// }
// TEST_F(RTDATest1, WhetherInfluenceJobSink) {
//   // arg order: jobCurr, jobChanged
//   EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(0, 1),
//   jobGroupMap, jobOrder, 5, 6, tasksInfo,
//                                        startTimeVector));
//   EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(1, 0),
//   jobGroupMap, jobOrder, 6, 7, tasksInfo,
//                                        startTimeVector));
//   EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(0, 1), JobCEC(0, 0),
//   jobGroupMap, jobOrder, 2, 3, tasksInfo,
//                                        startTimeVector));
//   EXPECT_FALSE(WhetherInfluenceJobSink(JobCEC(1, 0), JobCEC(2, 0),
//   jobGroupMap, jobOrder, 0, 1, tasksInfo,
//                                        startTimeVector));
//   EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(1, 0), JobCEC(2, 0),
//   jobGroupMap, jobOrder, 6, 7, tasksInfo,
//                                       startTimeVector));
//   EXPECT_TRUE(WhetherInfluenceJobSink(JobCEC(0, 0), JobCEC(2, 0),
//   jobGroupMap, jobOrder, 2, 3, tasksInfo,
//                                       startTimeVector));
// }

TEST_F(RTDATest3, WhetherInfluenceJobSimple) {
    EXPECT_TRUE(
        WhetherInfluenceJobSimple(JobCEC(2, 0), JobCEC(1, 1), jobGroupMap));
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
        startTimeVector << 123, 0, 1000, 0, 200, 400, 600, 800, 1000, 1200,
            1400, 1600, 1800;
        jobOrder = SFOrder(tasksInfo, startTimeVector);
    }
};

TEST_F(RTDATest4, whether_break_chain) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_,
                                "ReactionTimeObj");
    std::unordered_map<JobCEC, int> jobGroupMap_ =
        ExtractIndependentJobGroups(jobOrder, tasksInfo);
    JobCEC jobRelocate(2, 0);
    LLint startP = 3;
    LLint finishP = 4;
    EXPECT_FALSE(WhetherJobBreakChainRT(jobRelocate, startP, finishP,
                                        longestChain, dagTasks, jobOrder,
                                        tasksInfo));
    JobCEC sinkJob = longestChain[0].back();
    EXPECT_TRUE(WhetherInfluenceJobSink(sinkJob, jobRelocate, jobGroupMap_,
                                        jobOrder, startP, finishP, tasksInfo,
                                        startTimeVector));
}
class RTDATest8 : public RTDATest1 {
    void SetUp() override {
        std::string taskSetName = "test_n3_v45";
        SetUpTaskSet(taskSetName);
        startTimeVector = GenerateVectorDynamic(13);
        startTimeVector << 0, 0, 1000, 33, 200, 400, 600, 800, 1000, 1200, 1400,
            1600, 1800;
        jobOrder = SFOrder(tasksInfo, startTimeVector);
    }
};
TEST_F(RTDATest8, whether_break_chain) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_,
                                "ReactionTimeObj");
    std::unordered_map<JobCEC, int> jobGroupMap_ =
        ExtractIndependentJobGroups(jobOrder, tasksInfo);
    JobCEC jobRelocate(1, 1);
    LLint startP = 16;
    LLint finishP = 18;
    EXPECT_TRUE(WhetherJobBreakChainRT(jobRelocate, startP, finishP,
                                       longestChain, dagTasks, jobOrder,
                                       tasksInfo));
}
// *******************************************************

class RTDATest5 : public RTDATest1 {
    void SetUp() override {
        std::string taskSetName = "test_n3_v41";
        SetUpTaskSet(taskSetName);
        startTimeVector = GenerateVectorDynamic(17);
        startTimeVector << 0, 1205, 2000, 3000, 4000, 5000, 6000, 7000, 8000,
            9000, 678, 5000, 0, 3173, 4000, 6000, 8000;
        jobOrder = SFOrder(tasksInfo, startTimeVector);
    }
};
// TEST_F(RTDATest5, FindLongestCAChain) {
//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_); EXPECT_EQ(2, longestChain.size());

//   std::vector<JobCEC> chain0 = {JobCEC(2, 3), JobCEC(1, 2)}; // RT
//   std::vector<JobCEC> chain2 = {JobCEC(2, 2), JobCEC(1, 1)}; // RT&DA
//   AssertEqualVectorNoRepeat<JobCEC>(chain0, longestChain.longestChains_[0],
//   0, __LINE__); AssertEqualVectorNoRepeat<JobCEC>(chain2,
//   longestChain.longestChains_[1], 0, __LINE__);
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
// TODO:
// TEST_F(RTDATest6, FindLongestCAChain) {
//   LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
//   scheduleOptions.processorNum_); EXPECT_EQ(2, longestChain.size());

//   std::vector<JobCEC> chain0 = {JobCEC(1, 0), JobCEC(0, 1)}; // RT
//   std::vector<JobCEC> chain2 = {JobCEC(1, 0), JobCEC(0, 2)}; // RT&DA
//   AssertEqualVectorNoRepeat<JobCEC>(chain0, longestChain.longestChains_[0],
//   0, __LINE__); AssertEqualVectorNoRepeat<JobCEC>(chain2,
//   longestChain.longestChains_[1], 0, __LINE__);
// }
class RTDATest10 : public RTDATest1 {
    void SetUp() override {
        std::string taskSetName = "test_n3_v47";
        SetUpTaskSet(taskSetName);
        startTimeVector = GenerateVectorDynamic(12);
        startTimeVector << 78.4, 2156.8, 4235.2, 6313.6, 8000, 470.4, 5241.6,
            470.4, 2548.8, 4627.2, 6313.6, 8392;
        jobOrder = SFOrder(tasksInfo, startTimeVector);
        jobOrder.print();
    }
};

TEST_F(RTDATest10, find_longest_chain) {
    LongestCAChain longestChain(dagTasks, tasksInfo, jobOrder, startTimeVector,
                                scheduleOptions.processorNum_,
                                "ReactionTimeObj");
    // EXPECT_THAT(longestChain.size(), testing::Ge(10));
}
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}