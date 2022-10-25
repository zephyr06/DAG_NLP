#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/SFOrder.h"

using namespace OrderOptDAG_SPACE;

TEST(SFOrder, constructor_v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // int processorNum = 1;
    std::vector<uint> processorJobVec;
    VectorDynamic initial = GenerateVectorDynamic(3);
    initial << 3, 5, 0;
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder(tasksInfo, initial);
    EXPECT_LONGS_EQUAL(2, sfOrder.instanceOrder_[0].job.taskId);
    EXPECT_LONGS_EQUAL(2, sfOrder.instanceOrder_[1].job.taskId);
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[2].job.taskId);
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[3].job.taskId);
    EXPECT_LONGS_EQUAL(1, sfOrder.instanceOrder_[4].job.taskId);
    EXPECT_LONGS_EQUAL(1, sfOrder.instanceOrder_[5].job.taskId);

    JobCEC j00(0, 0);
    JobCEC j10(1, 0);
    JobCEC j20(2, 0);
    EXPECT_LONGS_EQUAL(0, sfOrder.GetJobStartInstancePosition(j20));
    EXPECT_LONGS_EQUAL(1, sfOrder.GetJobFinishInstancePosition(j20));
    EXPECT_LONGS_EQUAL(2, sfOrder.GetJobStartInstancePosition(j00));
    EXPECT_LONGS_EQUAL(3, sfOrder.GetJobFinishInstancePosition(j00));
    EXPECT_LONGS_EQUAL(4, sfOrder.GetJobStartInstancePosition(j10));
    EXPECT_LONGS_EQUAL(5, sfOrder.GetJobFinishInstancePosition(j10));
}

int main()
{
    TestResult tr;
    // make sure all tests have the correct setting
    NumCauseEffectChain = 1;
    return TestRegistry::runAllTests(tr);
}
