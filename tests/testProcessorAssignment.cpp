#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "sources/Optimization/ProcessorAssignment.h"

#include "sources/Utils/Parameters.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"

using namespace OrderOptDAG_SPACE;
using namespace GlobalVariablesDAGOpt;
using namespace ::testing;

class TestProcessorAssignment : public Test
{
public:
    int processorNum_;
    SFOrder sfOrder_;
    TaskSetInfoDerived tasksInfo_;
    std::vector<uint> processorJobVecRef_;
    std::vector<uint> processorJobVec_;
    void SetUp() override
    {
        processorNum_ = 2;
        processorJobVec_.clear();

        OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig");
        TaskSet tasks = dagTasks.tasks;
        tasksInfo_ = TaskSetInfoDerived(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo_, processorNum_, processorJobVecRef_);
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
