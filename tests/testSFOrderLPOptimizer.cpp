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

class TestSFOrderLPOptimizer : public Test
{
public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVecRef_;
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
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks_, tasksInfo_, processorNum_, processorJobVecRef_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        // PrintSchedule(tasksInfo_, initialSTV);
        // sfOrder_.print();
        scheduleOptions_.causeEffectChainNumber_ = 1;
        pSFOrderLPOptimizer_ = std::make_shared<SFOrderLPOptimizer>(dagTasks_);
    }
};

TEST_F(TestSFOrderLPOptimizer, CanBeInitializedSuccessfully)
{
    pSFOrderLPOptimizer_->setObjType(true);
    EXPECT_TRUE(true);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
