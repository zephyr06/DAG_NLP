

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <functional>

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/WorstSF_Fork.h"
#include "sources/Optimization/IndependentAnalysis.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/testMy.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

class RTDATest_n5_v84 : public ::testing::Test {
   protected:
    void SetUp() override {
        std::string taskSetName = "test_n5_v84";
        SetUpTaskSet(taskSetName);
        startTimeVector = ListSchedulingLFTPA(dagTasks, tasksInfo,
                                              scheduleOptions.processorNum_);
        jobOrder = SFOrder(tasksInfo, startTimeVector);
        jobOrder.print();
    }

    void SetUpTaskSet(std::string taskSet) {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +
                                     "TaskData/" + taskSet + ".csv",
                                 "orig");
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);

        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 1e6;
        scheduleOptions.sensorFusionTolerance_ = 1e6;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 1000;
        scheduleOptions.processorNum_ = 2;

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

    JobCEC job0 = JobCEC(0, 0);
    JobCEC job1 = JobCEC(0, 1);
    JobCEC job2 = JobCEC(0, 2);
    JobCEC job3 = JobCEC(0, 3);
};

TEST_F(RTDATest_n5_v84, WhetherImmediateForwardAdjacent_v1) {
    PrintSchedule(tasksInfo, startTimeVector);
    auto jobOrderRef = jobOrder;
    auto longestJobChains_ =
        LongestCAChain(dagTasks, tasksInfo, jobOrderRef, startTimeVector,
                       scheduleOptions.processorNum_, "ReactionTimeObj");
    auto jobGroupMap_ = ExtractIndependentJobGroups(jobOrderRef, tasksInfo);
    auto centralJob = FindCentralJobs(longestJobChains_, tasksInfo);
    std::vector<JobCEC> forwardJobs = FindForwardAdjacentJob(
        JobCEC(0, 1), jobOrderRef, tasksInfo, startTimeVector);
    EXPECT_EQ(3, forwardJobs.size());
    // Note: the order in the following test doesn't matter
    // EXPECT_EQ(JobCEC(0, 1), forwardJobs[0]);
    // EXPECT_EQ(JobCEC(1, 1), forwardJobs[1]);
    // EXPECT_EQ(JobCEC(3, 1), forwardJobs[2]);
    EXPECT_TRUE(ifExist(JobCEC(0, 1), forwardJobs));
    EXPECT_TRUE(ifExist(JobCEC(1, 1), forwardJobs));
    EXPECT_TRUE(ifExist(JobCEC(3, 1), forwardJobs));
    // auto activeJobs_ = FindActiveJobs(centralJob, jobOrderRef, tasksInfo,
    // startTimeVector); auto sth = FindForwardAdjacentJob(JobCEC(0, 0),
    // jobOrderRef, tasksInfo, startTimeVector); sth =
    // FindBackwardAdjacentJob(JobCEC(4, 0), jobOrderRef, tasksInfo,
    // startTimeVector); sth = FindBackwardAdjacentJob(JobCEC(4, 1),
    // jobOrderRef, tasksInfo, startTimeVector);
}
class RTDATest_n5_v85 : public RTDATest_n5_v84 {
    void SetUp() override {
        std::string taskSetName = "test_n5_v85";
        SetUpTaskSet(taskSetName);
        startTimeVector = ListSchedulingLFTPA(dagTasks, tasksInfo,
                                              scheduleOptions.processorNum_);
        jobOrder = SFOrder(tasksInfo, startTimeVector);
        jobOrder.print();
    }
};
TEST_F(RTDATest_n5_v85, WhetherImmediateForwardAdjacent_v1) {
    startTimeVector << 998.001, 3831, 998, 3000, 0.001, 2256, 5002, 1274, 3831,
        0, 2833, 5002;
    // TODO: analyze the longest DA job chain;
}

class ObjExperimentObjTest_n3_v57 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v57.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {2, 1, 0};
        dagTasks.chains_[0] = chain1;

        scheduleOptions.considerSensorFusion_ = 0;
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
// TEST_F(ObjExperimentObjTest_n3_v57, PredictInstanceIndex) {
//     VectorDynamic initialEstimate = GenerateVectorDynamic(5);
//     initialEstimate << 0, 1000, 0, 253, 1504;
//     SFOrder jobOrderRef(tasksInfo, initialEstimate);
//     jobOrderRef.print();

//     JobCEC jobRelocate(2, 0);
//     int startP = 1;
//     int finishP = 2;
//     EXPECT_EQ(-7, PredictInstanceIndexAfterRemoveInsertJob(
//                       jobOrderRef.GetJobStartInstancePosition(JobCEC(1, -1)),
//                       jobOrderRef.GetJobStartInstancePosition(jobRelocate),
//                       jobOrderRef.GetJobFinishInstancePosition(jobRelocate),
//                       startP, finishP));
//     EXPECT_EQ(-8, PredictInstanceIndexAfterRemoveInsertJob(
//                       jobOrderRef.GetJobFinishInstancePosition(JobCEC(2,
//                       -2)),
//                       jobOrderRef.GetJobStartInstancePosition(jobRelocate),
//                       jobOrderRef.GetJobFinishInstancePosition(jobRelocate),
//                       startP, finishP));
// }

TEST_F(ObjExperimentObjTest_n3_v57, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 0, 1000, 0, 253, 1504;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    JobCEC jobRelocate(2, 0);
    int startP = 1;
    int finishP = 2;
    auto longestJobChains_ = LongestCAChain(dagTasks, tasksInfo, jobOrderRef,
                                            initialEstimate, 2, "DataAgeObj");
    EXPECT_TRUE(WhetherJobBreakChainDA(jobRelocate, startP, finishP,
                                       longestJobChains_, dagTasks, jobOrderRef,
                                       tasksInfo));
}
class ObjExperimentObjTest_n5_v79 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {3, 2, 1, 0};
        dagTasks.chains_[0] = chain1;

        scheduleOptions.considerSensorFusion_ = 0;
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

TEST_F(ObjExperimentObjTest_n5_v79, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(19);
    initialEstimate << 0, 100, 200, 300, 400, 500, 0, 100, 200, 300, 400, 500,
        1, 201, 401, 12, 301, 1, 301;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    JobCEC jobRelocate(3, 0);
    int startP = 3;
    int finishP = 5;
    auto longestJobChains_ = LongestCAChain(dagTasks, tasksInfo, jobOrderRef,
                                            initialEstimate, 2, "DataAgeObj");
    EXPECT_TRUE(WhetherJobBreakChainDA(jobRelocate, startP, finishP,
                                       longestJobChains_, dagTasks, jobOrderRef,
                                       tasksInfo));
}

TEST_F(ObjExperimentObjTest_n5_v79, PredictInstanceIndex) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(19);
    initialEstimate << 0, 100, 200, 300, 400, 500, 0, 100, 200, 300, 400, 500,
        1, 201, 401, 12, 301, 1, 301;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    // jobs that do not change position
    EXPECT_EQ(10, PredictInstanceIndexAfterRemoveInsertJob(10, 0, 1, 2, 3));
    EXPECT_EQ(10, PredictInstanceIndexAfterRemoveInsertJob(10, 0, 1, 0, 1));
    EXPECT_EQ(5, PredictInstanceIndexAfterRemoveInsertJob(5, 1, 2, 3, 4));
    EXPECT_EQ(5, PredictInstanceIndexAfterRemoveInsertJob(5, 1, 2, 1, 2));

    EXPECT_EQ(5, PredictInstanceIndexAfterRemoveInsertJob(5, 4, 6, 4, 10));
    EXPECT_EQ(5, PredictInstanceIndexAfterRemoveInsertJob(5, 3, 4, 3, 4));

    // jobs that move forward
    EXPECT_EQ(3, PredictInstanceIndexAfterRemoveInsertJob(5, 3, 4, 9, 10));
    EXPECT_EQ(4, PredictInstanceIndexAfterRemoveInsertJob(5, 3, 8, 9, 10));
    EXPECT_EQ(4, PredictInstanceIndexAfterRemoveInsertJob(5, 3, 4, 3, 10));

    // jobs that move backward
    EXPECT_EQ(7, PredictInstanceIndexAfterRemoveInsertJob(5, 6, 7, 1, 2));
    EXPECT_EQ(6, PredictInstanceIndexAfterRemoveInsertJob(5, 3, 6, 4, 5));
}

class ObjExperimentObjTest_n5_v86 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v86.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        dagTasks.chains_ = {{4, 3, 0}, {4, 2, 1}};

        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    ScheduleOptions scheduleOptions;
};

TEST_F(ObjExperimentObjTest_n5_v86, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(15);
    initialEstimate << 1900, 2000, 4236, 2000, 4236, 929, 3165, 0, 1791, 2853,
        3000, 4127, 5089, 663.999, 3000;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    auto longestJobChains_ =
        LongestCAChain(dagTasks, tasksInfo, jobOrderRef, initialEstimate,
                       scheduleOptions.processorNum_, "DataAgeObj");
    auto centralJob = FindCentralJobs(longestJobChains_, tasksInfo);
    auto activeJobs =
        FindActiveJobs(centralJob, jobOrderRef, tasksInfo, initialEstimate);
    std::vector<JobCEC> jobs = activeJobs.GetJobs();
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 2), jobs));
}

class ObjExperimentObjTest_n3_v59 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v59.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        dagTasks.chains_ = {{1, 0}};

        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    ScheduleOptions scheduleOptions;
};

TEST_F(ObjExperimentObjTest_n3_v59, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(16);
    initialEstimate << 327, 1000, 2000, 3697, 4000, 5697, 6000, 7697, 8536,
        9209, 629, 3326, 5326, 7047, 8838, 999;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    auto longestJobChains_ =
        LongestCAChain(dagTasks, tasksInfo, jobOrderRef, initialEstimate,
                       scheduleOptions.processorNum_, "DataAgeObj");
    auto centralJob = FindCentralJobs(longestJobChains_, tasksInfo);
    auto activeJobs =
        FindActiveJobs(centralJob, jobOrderRef, tasksInfo, initialEstimate);
    std::vector<JobCEC> jobs = activeJobs.GetJobs();
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), jobs));
}
class ObjExperimentObjTest_n3_v60 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v60.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        dagTasks.chains_ = {{2, 0}, {2, 1}};

        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    ScheduleOptions scheduleOptions;
};

TEST_F(ObjExperimentObjTest_n3_v60, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(13);
    initialEstimate << 0, 2235, 4000, 6235, 8000, 10000, 12235, 14000, 16000,
        18000, 2189, 1202, 11248;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    auto longestJobChains_ =
        LongestCAChain(dagTasks, tasksInfo, jobOrderRef, initialEstimate,
                       scheduleOptions.processorNum_, "DataAgeObj");
    auto centralJob = FindCentralJobs(longestJobChains_, tasksInfo);
    auto activeJobs =
        FindActiveJobs(centralJob, jobOrderRef, tasksInfo, initialEstimate);
    std::vector<JobCEC> jobs = activeJobs.GetJobs();
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 3), jobs));
}
class ObjExperimentObjTest_n5_v88 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v88.csv",
            "orig");  // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        dagTasks.chains_ = {{3, 1}, {4, 2, 0}};

        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    ScheduleOptions scheduleOptions;
};

TEST_F(ObjExperimentObjTest_n5_v88, WhetherJobBreakChainDA) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(32);
    initialEstimate << 0, 117, 235, 387, 400, 560, 614, 787, 800, 902, 15, 268,
        413, 690, 814, 0, 103, 200, 336, 400, 546, 600, 773, 800, 900, 130, 573,
        14, 247, 414, 684, 813;
    SFOrder jobOrderRef(tasksInfo, initialEstimate);
    jobOrderRef.print();
    auto longestJobChains =
        LongestCAChain(dagTasks, tasksInfo, jobOrderRef, initialEstimate,
                       scheduleOptions.processorNum_, "DataAgeObj");
    auto centralJob = FindCentralJobs(longestJobChains, tasksInfo);
    auto activeJobs =
        FindActiveJobs(centralJob, jobOrderRef, tasksInfo, initialEstimate);
    std::vector<JobCEC> jobs = activeJobs.GetJobs();
    // EXPECT_TRUE(ifExist<JobCEC>(JobCEC(2, 9), jobs));
    LLint startP = 61, finishP = 63;
    JobCEC job(2, 9);
    EXPECT_TRUE(
        ifExist<JobCEC>(JobCEC(2, 9 - 10), longestJobChains.longestChains_[1]));
    EXPECT_TRUE(WhetherJobBreakChainDA(job, startP, finishP, longestJobChains,
                                       dagTasks, jobOrderRef, tasksInfo));
}

class TestSFOrderLPOptimizer_da_n3_v61 : public ::testing::Test {
   public:
    OrderOptDAG_SPACE::DAG_Model dagTasks;
    int processorNum;
    SFOrder sfOrder;
    TaskSetInfoDerived tasksInfo;
    std::vector<uint> processorJobVec;
    OptimizeSF::ScheduleOptions scheduleOptions;

    void SetUp() override {
        processorNum = 2;
        dagTasks =
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v61.csv", "orig", 1);
        TaskSet tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks, tasksInfo, processorNum, processorJobVec);
        sfOrder = SFOrder(tasksInfo, initialSTV);
        PrintSchedule(tasksInfo, initialSTV);
        // sfOrder.print();
        scheduleOptions.causeEffectChainNumber_ = 1;
    }
};
TEST_F(TestSFOrderLPOptimizer_da_n3_v61, WorstSF_JobFork) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 15;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    EXPECT_EQ(1, worst_sf_fork.size());
    EXPECT_EQ(JobCEC(2, 0), worst_sf_fork[0].sink_job);
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 1), worst_sf_fork[0].source_jobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), worst_sf_fork[0].source_jobs));
}

TEST_F(TestSFOrderLPOptimizer_da_n3_v61, FindCentralJobs_sf_v1) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 15;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    CentralJobs central_jobs = FindCentralJobs(worst_sf_fork, tasksInfo);
    EXPECT_EQ(0, central_jobs.forwardJobs.size());
    EXPECT_EQ(2, central_jobs.backwardJobs.size());
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 1), central_jobs.backwardJobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), central_jobs.backwardJobs));
}
TEST_F(TestSFOrderLPOptimizer_da_n3_v61, FindActiveJobs_sf_v2) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 11;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    CentralJobs central_jobs = FindCentralJobs(worst_sf_fork, tasksInfo);
    auto activeJobs =
        FindActiveJobs(central_jobs, sfOrder, tasksInfo, initialSTV);
    EXPECT_EQ(0, central_jobs.forwardJobs.size());
    EXPECT_EQ(2, central_jobs.backwardJobs.size());
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(0, 1)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(1, 0)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(2, 0)) > 0);
    EXPECT_TRUE(activeJobs.activeJobs[0].direction_backward);
    EXPECT_FALSE(activeJobs.activeJobs[0].direction_forward);
}

TEST_F(TestSFOrderLPOptimizer_da_n3_v61, WorstSF_JobFork_v2) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 5;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    EXPECT_EQ(1, worst_sf_fork.size());
    EXPECT_EQ(JobCEC(2, 0), worst_sf_fork[0].sink_job);
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 0), worst_sf_fork[0].source_jobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), worst_sf_fork[0].source_jobs));
}

class TestSFOrderLPOptimizer_da_n3_v63 : public ::testing::Test {
   public:
    OrderOptDAG_SPACE::DAG_Model dagTasks;
    int processorNum;
    SFOrder sfOrder;
    TaskSetInfoDerived tasksInfo;
    std::vector<uint> processorJobVec;
    OptimizeSF::ScheduleOptions scheduleOptions;

    void SetUp() override {
        processorNum = 2;
        dagTasks =
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v63.csv", "orig", 1);
        TaskSet tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks, tasksInfo, processorNum, processorJobVec);
        sfOrder = SFOrder(tasksInfo, initialSTV);
        PrintSchedule(tasksInfo, initialSTV);
        // sfOrder.print();
        scheduleOptions.causeEffectChainNumber_ = 1;
    }
};

TEST_F(TestSFOrderLPOptimizer_da_n3_v63, WorstSF_JobFork) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 100, 0, 50, 100;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    EXPECT_EQ(2, worst_sf_fork.size());
    EXPECT_EQ(JobCEC(2, 0), worst_sf_fork[0].sink_job);
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, -1), worst_sf_fork[0].source_jobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), worst_sf_fork[0].source_jobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(0, 0), worst_sf_fork[1].source_jobs));
    EXPECT_TRUE(ifExist<JobCEC>(JobCEC(1, 0), worst_sf_fork[1].source_jobs));
}
TEST_F(TestSFOrderLPOptimizer_da_n3_v63, FindActiveJobs_sf_v2) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 100, 0, 50, 100;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    WorstSF_JobFork worst_sf_fork(dagTasks, tasksInfo, sfOrder, initialSTV, 2);
    CentralJobs central_jobs = FindCentralJobs(worst_sf_fork, tasksInfo);
    auto activeJobs =
        FindActiveJobs(central_jobs, sfOrder, tasksInfo, initialSTV);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(0, 0)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(0, 1)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(1, 0)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(2, 0)) > 0);
    EXPECT_TRUE(activeJobs.jobRecord.count(JobCEC(2, 1)) > 0);
    EXPECT_TRUE(activeJobs.activeJobs[0].direction_backward);
    EXPECT_FALSE(activeJobs.activeJobs[0].direction_forward);
}
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}