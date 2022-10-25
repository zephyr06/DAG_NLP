#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"

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

TEST(SFOrder, sched_v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 1;

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder(tasksInfo, initial);
    VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder);
    PrintSchedule(tasksInfo, initialSTV);
}

TEST(SFOrder, insert_erase)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    SFOrder sfOrder(tasksInfo, initial);

    JobCEC j00(0, 0);
    JobCEC j10(1, 0);
    JobCEC j20(2, 0);
    sfOrder.RemoveJob(j00);
    EXPECT_LONGS_EQUAL(4, sfOrder.size());
    sfOrder.InsertStart(j00, 2);
    sfOrder.InsertFinish(j00, 3);
    VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder);
    EXPECT(assert_equal(initial, initialSTV));
}
TEST(WhetherSkipInsertStart_finish, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    SFOrder sfOrder(tasksInfo, initial);
    JobCEC j00(0, 0);
    JobCEC j10(1, 0);
    JobCEC j20(2, 0);
    for (uint i = 0; i < 6; i++)
    {
        std::cout << "Test i: " << i << std::endl;
        EXPECT(!WhetherSkipInsertStart(j00, i, tasksInfo, sfOrder));
        EXPECT(!WhetherSkipInsertFinish(j00, i, tasksInfo, sfOrder));
        EXPECT(!WhetherSkipInsertStart(j10, i, tasksInfo, sfOrder));
        EXPECT(!WhetherSkipInsertFinish(j10, i, tasksInfo, sfOrder));
        EXPECT(!WhetherSkipInsertStart(j20, i, tasksInfo, sfOrder));
        EXPECT(!WhetherSkipInsertFinish(j20, i, tasksInfo, sfOrder));
    }
}
TEST(WhetherSkipInsertStart_finish, v2)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v20.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder(tasksInfo, initial);
    JobCEC j00(0, 0);
    JobCEC j01(0, 1);
    JobCEC j02(0, 2);
    JobCEC j03(0, 3);
    JobCEC j11(1, 1);
    JobCEC j20(2, 0);
    EXPECT(!WhetherSkipInsertStart(j00, 0, tasksInfo, sfOrder));
    // EXPECT(WhetherSkipInsertStart(j00, 6, tasksInfo, sfOrder));
    EXPECT(WhetherSkipInsertStart(j00, 7, tasksInfo, sfOrder));
    EXPECT(WhetherSkipInsertStart(j00, 8, tasksInfo, sfOrder));
    // EXPECT(WhetherSkipInsertFinish(j00, 6, tasksInfo, sfOrder));
    // EXPECT(WhetherSkipInsertFinish(j00, 7, tasksInfo, sfOrder));
    EXPECT(WhetherSkipInsertFinish(j00, 9, tasksInfo, sfOrder));
}

TEST(SFOrder, opt_v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 1;
    ScheduleResult sRes = ScheduleDAGModel(dagTasks, processorNum);
    PrintSchedule(tasksInfo, sRes.startTimeVector_);
}

int main()
{
    TestResult tr;
    // make sure all tests have the correct setting
    NumCauseEffectChain = 1;
    return TestRegistry::runAllTests(tr);
}
