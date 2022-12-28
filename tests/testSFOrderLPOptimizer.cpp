#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "sources/Optimization/SFOrderLPOptimizer.h"

#include "sources/Utils/Parameters.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/OrderScheduler.h"

using namespace OrderOptDAG_SPACE;
using namespace GlobalVariablesDAGOpt;
using namespace ::testing;

class TestSFOrderLPOptimizerBasicFunction : public Test
{
public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    std::shared_ptr<SFOrderLPOptimizer> pSFOrderLPOptimizer_;
    void SetUp() override
    {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig", 1);
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks_, tasksInfo_, processorNum_, processorJobVec_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        // PrintSchedule(tasksInfo_, initialSTV);
        // sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(dagTasks_, sfOrder_);
        pSFOrderLPOptimizer_->Init();
    }
    void TearDown() override
    {
        pSFOrderLPOptimizer_->ClearMemory();
    }
};

TEST_F(TestSFOrderLPOptimizerBasicFunction, CanBeInitializedSuccessfully)
{
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->Init());
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->ClearMemory());
}
TEST_F(TestSFOrderLPOptimizerBasicFunction, WillAddCorrectLPVariableNumber)
{
    pSFOrderLPOptimizer_->AddVariables();
    EXPECT_THAT((long)pSFOrderLPOptimizer_->varArray_.getSize(), Eq(tasksInfo_.variableDimension));
}
TEST_F(TestSFOrderLPOptimizerBasicFunction, CanReadAndSetCorrectProcessorAssignment)
{
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->setProcessorJobVec(processorJobVec_));
}

class TestSFOrderLPOptimizer : public Test
{
public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    std::shared_ptr<SFOrderLPOptimizer> pSFOrderLPOptimizer_;
    void SetUp() override
    {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig", 1);
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks_, tasksInfo_, processorNum_, processorJobVec_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        // PrintSchedule(tasksInfo_, initialSTV);
        // sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(dagTasks_, sfOrder_);
        pSFOrderLPOptimizer_->Init();
        pSFOrderLPOptimizer_->AddVariables();
        pSFOrderLPOptimizer_->AddDDLConstraints();
        pSFOrderLPOptimizer_->setProcessorJobVec(processorJobVec_);
    }
    void TearDown() override
    {
        pSFOrderLPOptimizer_->ClearMemory();
    }
};

TEST_F(TestSFOrderLPOptimizer, CanAddDBFConstraintsUsingSFOrder)
{
    EXPECT_NO_THROW(pSFOrderLPOptimizer_->AddDBFConstraints());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
