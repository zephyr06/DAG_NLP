#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "sources/Optimization/ProcessorAssignment.h"

#include "sources/Utils/Parameters.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/OrderScheduler.h"

using namespace OrderOptDAG_SPACE;
using namespace GlobalVariablesDAGOpt;
using namespace ::testing;

class TestProcessorAssignment : public Test
{
public:
    OrderOptDAG_SPACE::DAG_Model dagTasks_;
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVecRef_;
    std::vector<uint> processorJobVec_;
    OptimizeSF::ScheduleOptions scheduleOptions_;
    void SetUp() override
    {
        processorNum_ = 2;
        processorJobVec_.clear();
        dagTasks_ = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig");
        TaskSet tasks = dagTasks_.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks_, tasksInfo_, processorNum_, processorJobVecRef_);
        sfOrder_ = SFOrder(tasksInfo_, initialSTV);
        // PrintSchedule(tasksInfo_, initialSTV);
        // sfOrder_.print();
    }
    bool HaveSameProcessorAssignment(std::vector<uint> &v1, std::vector<uint> &v2)
    {
        if (v1.size() != v2.size())
            return false;
        for (auto i = 0u; i < v1.size(); i++)
            if (v1.at(i) != v2.at(i))
                return false;
        return true;
    }
};

TEST_F(TestProcessorAssignment, ReadRightArguments)
{
    EXPECT_NO_THROW(ProcessorAssignment::AssignProcessor(tasksInfo_, sfOrder_, processorNum_, processorJobVec_));
}
TEST_F(TestProcessorAssignment, CanAssignProcessor)
{
    EXPECT_THAT(ProcessorAssignment::AssignProcessor(tasksInfo_, sfOrder_, processorNum_, processorJobVec_), IsTrue());
}
TEST_F(TestProcessorAssignment, WillAssignCorrectProcessor)
{
    EXPECT_THAT(ProcessorAssignment::AssignProcessor(tasksInfo_, sfOrder_, processorNum_, processorJobVec_), IsTrue());
    EXPECT_THAT(HaveSameProcessorAssignment(processorJobVec_, processorJobVecRef_), IsTrue());
}
TEST_F(TestProcessorAssignment, WillFailProcessorAssignment)
{
    VectorDynamic initialSTV = GenerateVectorDynamic(tasksInfo_.variableDimension);
    initialSTV << 0, 10, 20, 0, 0;
    sfOrder_ = SFOrder(tasksInfo_, initialSTV);
    EXPECT_THAT(ProcessorAssignment::AssignProcessor(tasksInfo_, sfOrder_, processorNum_, processorJobVec_), IsFalse());
}
TEST_F(TestProcessorAssignment, WillAssignProcessorSameAsSimpleOrderScheduler)
{
    VectorDynamic initialSTV = GenerateVectorDynamic(tasksInfo_.variableDimension);
    initialSTV << 5, 10, 20, 1, 0;
    sfOrder_ = SFOrder(tasksInfo_, initialSTV);
    EXPECT_THAT(ProcessorAssignment::AssignProcessor(tasksInfo_, sfOrder_, processorNum_, processorJobVec_), IsTrue());
    scheduleOptions_.processorNum_ = processorNum_;
    SimpleOrderScheduler::schedule(dagTasks_, tasksInfo_, scheduleOptions_, sfOrder_, processorJobVecRef_);
    EXPECT_THAT(HaveSameProcessorAssignment(processorJobVec_, processorJobVecRef_), IsTrue());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
