#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sources/Factors/ObjSensorFusion.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/SFOrderLPOptimizer.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Utils/Parameters.h"

using namespace OrderOptDAG_SPACE;
using namespace GlobalVariablesDAGOpt;
using namespace ::testing;

class TestSFOrderLPOptimizerBasicFunction : public Test {
   public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    std::shared_ptr<SFOrderLPOptimizer> pSFOrderLPOptimizer_;
    void SetUp() override {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ =
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig", 1);
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks_, tasksInfo_, processorNum_, processorJobVec_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        // PrintSchedule(tasksInfo_, initialSTV);
        sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(
            dagTasks_, sfOrder_, processorNum_, "ReactionTimeObj");
        pSFOrderLPOptimizer_->Init();
    }
    void TearDown() override { pSFOrderLPOptimizer_->ClearCplexMemory(); }
};

TEST_F(TestSFOrderLPOptimizerBasicFunction, CanBeInitializedSuccessfully) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->Init());
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->ClearCplexMemory());
}
TEST_F(TestSFOrderLPOptimizerBasicFunction, WillAddCorrectLPVariableNumber) {
    pSFOrderLPOptimizer_->AddVariables();
    EXPECT_THAT((long)pSFOrderLPOptimizer_->varArray_.getSize(),
                Eq(tasksInfo_.variableDimension));
}
TEST_F(TestSFOrderLPOptimizerBasicFunction,
       CanReadAndSetCorrectProcessorAssignment) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->setProcessorJobVec(processorJobVec_));
}

class TestSFOrderLPOptimizer : public Test {
   public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    std::shared_ptr<SFOrderLPOptimizer> pSFOrderLPOptimizer_;
    void SetUp() override {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ =
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig", 1);
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks_, tasksInfo_, processorNum_, processorJobVec_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        PrintSchedule(tasksInfo_, initialSTV);
        sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(
            dagTasks_, sfOrder_, processorNum_, "ReactionTimeObj");
        pSFOrderLPOptimizer_->Init();
        pSFOrderLPOptimizer_->AddVariables();
        pSFOrderLPOptimizer_->AddDDLConstraints();
        pSFOrderLPOptimizer_->setProcessorJobVec(processorJobVec_);
    }
    void TearDown() override { pSFOrderLPOptimizer_->ClearCplexMemory(); }
};

TEST_F(TestSFOrderLPOptimizer, CanAddDBFConstraintsUsingSFOrder) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->AddDBFConstraints());
}
TEST_F(TestSFOrderLPOptimizer, CanAddNormalObjectives) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->AddDBFConstraints());
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->AddObjectives());
}
TEST_F(TestSFOrderLPOptimizer, CanDoBasicOptimization) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->Optimize(processorJobVec_));
}
TEST_F(TestSFOrderLPOptimizer, CanDoCorrectOptimization) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->Optimize(processorJobVec_));
    VectorDynamic stvOptimized =
        pSFOrderLPOptimizer_->getOptimizedStartTimeVector();
    // PrintSchedule(tasksInfo_, stvOptimized);
    VectorDynamic stvExpected = GenerateVectorDynamic(5);
    stvExpected << 9, 19, 29, 14, 0;
    EXPECT_THAT(stvOptimized, Eq(stvExpected));
}

class TestSFOrderLPOptimizer_da : public Test {
   public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    std::shared_ptr<SFOrderLPOptimizer> pSFOrderLPOptimizer_;
    void SetUp() override {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ =
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig", 1);
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks_, tasksInfo_, processorNum_, processorJobVec_);
        initialSTV(4) = 10;
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        PrintSchedule(tasksInfo_, initialSTV);
        sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(
            dagTasks_, sfOrder_, processorNum_, "DataAgeObj");
        pSFOrderLPOptimizer_->Init();
        pSFOrderLPOptimizer_->AddVariables();
        pSFOrderLPOptimizer_->AddDDLConstraints();
        pSFOrderLPOptimizer_->setProcessorJobVec(processorJobVec_);
    }
    void TearDown() override { pSFOrderLPOptimizer_->ClearCplexMemory(); }
};
TEST_F(TestSFOrderLPOptimizer_da, CanDoCorrectOptimization) {
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->Optimize(processorJobVec_));
    VectorDynamic stvOptimized =
        pSFOrderLPOptimizer_->getOptimizedStartTimeVector();
    std::cout << stvOptimized << "\n";
    // PrintSchedule(tasksInfo_, stvOptimized);
    VectorDynamic stvExpected = GenerateVectorDynamic(5);
    stvExpected << 4, 10, 29, 5, 10;
    EXPECT_THAT(stvOptimized, Eq(stvExpected));
}
class TestSFOrderLPOptimizer_da_n3_v56 : public Test {
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
            ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v56.csv", "orig", 1);
        TaskSet tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(
            dagTasks, tasksInfo, processorNum, processorJobVec);
        sfOrder = SFOrder(tasksInfo, initialSTV);
        PrintSchedule(tasksInfo, initialSTV);
        sfOrder.print();
        scheduleOptions.causeEffectChainNumber_ = 1;
    }
};
TEST_F(TestSFOrderLPOptimizer_da_n3_v56, CanDoCorrectOptimization) {
    SFOrderLPOptimizer pSFOrderLPOptimizer(dagTasks, sfOrder, processorNum,
                                           "DataAgeObj");
    EXPECT_NO_THROW(pSFOrderLPOptimizer.Optimize(processorJobVec));
    VectorDynamic stvOptimized =
        pSFOrderLPOptimizer.getOptimizedStartTimeVector();
    std::cout << stvOptimized << "\n";
    PrintSchedule(tasksInfo, stvOptimized);
    EXPECT_TRUE(ExamBasic_Feasibility(dagTasks, tasksInfo, stvOptimized,
                                      processorJobVec,
                                      scheduleOptions.processorNum_));

    // VectorDynamic stvExpected = GenerateVectorDynamic(5);
    // stvExpected << 4, 10, 29, 5, 10;
    // EXPECT_THAT(stvOptimized, Eq(stvExpected));
}

class TestSFOrderLPOptimizer_da_n3_v61 : public Test {
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
TEST_F(TestSFOrderLPOptimizer_da_n3_v61, CanDoCorrectOptimization) {
    VectorDynamic initialSTV =
        ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    initialSTV << 0, 10, 1, 15;
    sfOrder = SFOrder(tasksInfo, initialSTV);
    sfOrder.print();
    SFOrderLPOptimizer pSFOrderLPOptimizer(dagTasks, sfOrder, processorNum,
                                           "SensorFusionObj");
    EXPECT_NO_THROW(pSFOrderLPOptimizer.Optimize(processorJobVec));
    VectorDynamic stvOptimized =
        pSFOrderLPOptimizer.getOptimizedStartTimeVector();
    std::cout << stvOptimized << "\n";
    PrintSchedule(tasksInfo, stvOptimized);
    EXPECT_TRUE(ExamBasic_Feasibility(dagTasks, tasksInfo, stvOptimized,
                                      processorJobVec,
                                      scheduleOptions.processorNum_));
    EXPECT_EQ(
        stvOptimized(2) + 2,
        stvOptimized(
            1));  // job 0-0 and job 1-0 are the last-reading job of job 2-0
    // EXPECT_THAT(stvOptimized(0), ::testing::Le(stvOptimized(3)));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
