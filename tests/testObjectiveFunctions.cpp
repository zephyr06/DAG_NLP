#include <gtest/gtest.h>

#include "sources/Factors/ObjectiveFunctions.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/TaskModel/DAG_Model.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;
class ObjExperimentObjTest1 : public ::testing::Test {
   protected:
    void SetUp() override {
        dagTasks = ReadDAG_Tasks(
            GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v1.csv",
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

TEST_F(ObjExperimentObjTest1, constructor) { EXPECT_EQ(tasks.size(), 5); }
TEST_F(ObjExperimentObjTest1, RTDAEvaluate1) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4,
        5;  // the chain starts at 0 and ends at 3+12+200+200=415

    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                          scheduleOptions),
              (414 + 414) * 2 * 0.5 + 414 +
                  414);  // There are two job-level chains even though
                         // hyper-period equals 200

    EXPECT_EQ(ReactionTimeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                        scheduleOptions),
              (414 + 414) * 0.5 + 414);
    EXPECT_EQ(DataAgeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                   scheduleOptions),
              (414 + 414) * 0.5 + 414);
}
TEST_F(ObjExperimentObjTest1, SFEvaluate1) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    // EXPECT_EQ(RTSS21ICObj::EvaluateRTDA(dagTasks, tasksInfo, initialEstimate,
    //                                     scheduleOptions),
    //           (414 + 414) * 2 * 0.5 +
    //               (414 + 414) * 10);  // There are two job-level chains even
    //                                   // though hyper-period equals 200
    // EXPECT_EQ(RTSS21ICObj::EvaluateSF(dagTasks, tasksInfo, initialEstimate,
    //                                   scheduleOptions),
    //           4 * 0.5 + 4 * 10);
    // EXPECT_EQ(RTSS21ICObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
    //                                 scheduleOptions),
    //           42 + 9108);
    dagTasks.sf_forks_.push_back(SF_Fork({0, 1}, 2));
    EXPECT_EQ(SensorFusionObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                        scheduleOptions),
              2);
}

TEST(test_n3_v18, nonWorkConserveCase) {
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv",
        "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    ScheduleOptions scheduleOptions;
    scheduleOptions.causeEffectChainNumber_ = 1;
    scheduleOptions.processorNum_ = 2;

    VectorDynamic initial =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 17, 18, 1;
    SFOrder sfOrder(tasksInfo, initial);

    VectorDynamic initialSTV =
        SFOrderScheduling(dagTasks.tasks, tasksInfo, processorNum, sfOrder);
    PrintSchedule(tasksInfo, initialSTV);
    VectorDynamic expect = GenerateVectorDynamic(4);
    expect << 0, 10, 11, 1;
    EXPECT_TRUE(assert_equal(expect, initialSTV));
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initialSTV,
                                          scheduleOptions),
              (4 + 14) + (4 + 4 + 14 + 4 + 4) * 0.5);

    initial << 2, 10, 0, 0;
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder2(tasksInfo, initial);
    sfOrder2.print();
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial,
                                          scheduleOptions),
              (21 + 13) + (21 + 13 + 13 + 21) * 0.5);
}
TEST(Obj, RTDA_v1) {
    DAG_Model dagTasks = ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv",
        "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    VectorDynamic initial =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 10, 1,
        1;  // 3 job-level chains, whose RTDA are (4, 4), (14, -1), (4, 4)

    ScheduleOptions scheduleOptions;
    scheduleOptions.causeEffectChainNumber_ = 1;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;
    scheduleOptions.freshTol_ = 0;

    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial,
                                          scheduleOptions),
              33);
    // EXPECT_EQ(
    //     RTSS21ICObj::Evaluate(dagTasks, tasksInfo, initial, scheduleOptions),
    //     180 + 30 * 0.5);

    // EXPECT_LONGS_EQUAL(18, status.ReadObj());
    // EXPECT_DOUBLES_EQUAL(32.5, status.ObjWeighted(), 0.1);
    // EXPECT_LONGS_EQUAL(18, status.ObjBarrier());

    // EXPECT_LONGS_EQUAL(18, status2.ReadObj());
    // EXPECT_LONGS_EQUAL(18, status2.ObjBarrier());
    // EXPECT_DOUBLES_EQUAL(32.5, status2.ObjWeighted(), 0.1);
    scheduleOptions.freshTol_ = 100;
    EXPECT_EQ(RTDAExperimentObj::Evaluate(dagTasks, tasksInfo, initial,
                                          scheduleOptions),
              33);
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
TEST_F(ObjExperimentObjTest_n5_v2, RTDAEvaluate1) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(6);
    initialEstimate << 1, 100, 2, 3, 4,
        5;  // the chain starts at 0 and ends at 3+12+200+200=415
    // Within a hyper-period, RT is 414, 315, DA is 315

    EXPECT_EQ(ReactionTimeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                        scheduleOptions),
              (414 + 315 + 414) * 0.5 + 414);
    EXPECT_EQ(ReactionTimeObj::TrueObj(dagTasks, tasksInfo, initialEstimate,
                                       scheduleOptions),
              414);
    EXPECT_EQ(DataAgeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
                                   scheduleOptions),
              (315) * 0.5 + 315);
    EXPECT_EQ(DataAgeObj::TrueObj(dagTasks, tasksInfo, initialEstimate,
                                  scheduleOptions),
              315);
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

TEST_F(ObjExperimentObjTest_n5_v79, DAEvaluate1) {
    VectorDynamic initialEstimate = GenerateVectorDynamic(19);
    initialEstimate << 0, 100, 200, 300, 400, 500, 0, 100, 200, 300, 400, 500,
        1, 201, 401, 12, 301, 1, 301;

    // EXPECT_EQ(ReactionTimeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
    //                                     scheduleOptions),
    //           (414 + 315 + 414) * 0.5 + 414);
    // EXPECT_EQ(ReactionTimeObj::TrueObj(dagTasks, tasksInfo, initialEstimate,
    //                                    scheduleOptions),
    //           414);
    // EXPECT_EQ(DataAgeObj::Evaluate(dagTasks, tasksInfo, initialEstimate,
    //                                scheduleOptions),
    //           (315) * 0.5 + 315);
    EXPECT_EQ(DataAgeObj::TrueObj(dagTasks, tasksInfo, initialEstimate,
                                  scheduleOptions),
              600);
    initialEstimate << 0, 103, 200, 300, 400, 500, 0, 103, 200, 300, 400, 500,
        10, 201, 401, 1, 301, 10, 301;
    EXPECT_EQ(DataAgeObj::TrueObj(dagTasks, tasksInfo, initialEstimate,
                                  scheduleOptions),
              500);
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
TEST_F(TestSFOrderLPOptimizer_da_n3_v61, SF_Fork_analyze) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 15;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
}

class TestSFOrderLPOptimizer_da_n3_v66 : public ::testing::Test {
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
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v66.csv", "orig", 1);
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
TEST_F(TestSFOrderLPOptimizer_da_n3_v66, SF_Obj) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 960, 3960, 0, 1000, 2000, 3368, 4000, 5000, 0, 2033, 4000;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    EXPECT_EQ(5632 - 4000,
              SensorFusionObj::TrueObj(dagTasks, tasksInfo, initialSTV,
                                       scheduleOptions));
}
class TestSFOrderLPOptimizer_da_n3_v67 : public ::testing::Test {
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
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v67.csv", "orig", 1);
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
TEST_F(TestSFOrderLPOptimizer_da_n3_v67, SF_Obj) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 954, 4001, 1855, 2000, 4902, 236, 1000, 2000, 3236, 4000,
        5236;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    EXPECT_EQ(2145 - 2000,
              SensorFusionObj::TrueObj(dagTasks, tasksInfo, initialSTV,
                                       scheduleOptions));
}
class TestSFOrderLPOptimizer_da_n3_v68 : public ::testing::Test {
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
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v68.csv", "orig", 1);
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
TEST_F(TestSFOrderLPOptimizer_da_n3_v68, SF_Obj) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 551, 0, 1741, 2000, 0, 1271, 2000;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    EXPECT_EQ(259 - (1741 - 3000),
              SensorFusionObj::TrueObj(dagTasks, tasksInfo, initialSTV,
                                       scheduleOptions));
}
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}