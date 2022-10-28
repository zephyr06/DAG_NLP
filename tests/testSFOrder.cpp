#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;

TEST(SFOrder, constructor_v3)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v24.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    std::vector<uint> processorJobVec;
    VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initialSTV << 440, 0, 3560, 5560, 7560, 9560, 1047, 3047, 5047, 7047,
        9047;
    PrintSchedule(tasksInfo, initialSTV);
    SFOrder sfOrder(tasksInfo, initialSTV);
    sfOrder.print();
}

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
TEST(SFOrder, ReadObj)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 17, 18, 1;
    SFOrder sfOrder(tasksInfo, initial);
    sfOrder.print();
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[0].job.taskId);
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[1].job.taskId);
    EXPECT_LONGS_EQUAL(2, sfOrder.instanceOrder_[2].job.taskId);
    EXPECT_LONGS_EQUAL(2, sfOrder.instanceOrder_[3].job.taskId);
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[4].job.taskId);
    EXPECT_LONGS_EQUAL(0, sfOrder.instanceOrder_[5].job.taskId);
    EXPECT_LONGS_EQUAL(1, sfOrder.instanceOrder_[6].job.taskId);
    EXPECT_LONGS_EQUAL(1, sfOrder.instanceOrder_[7].job.taskId);

    VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder);
    PrintSchedule(tasksInfo, initialSTV);
    VectorDynamic expect = initialSTV;
    expect << 0, 10, 11, 1;
    EXPECT(assert_equal(expect, initialSTV));
    IterationStatus statusPrev(dagTasks, tasksInfo, sfOrder, processorNum);
    EXPECT_LONGS_EQUAL(4 + 14, statusPrev.ReadObj());

    initial << 2, 10, 0, 0;
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder2(tasksInfo, initial);
    sfOrder2.print();
    IterationStatus statusPrev2(dagTasks, tasksInfo, sfOrder2, processorNum);
    EXPECT_LONGS_EQUAL(21 + 13, statusPrev2.ReadObj());
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
    EXPECT_LONGS_EQUAL(0, sfOrder.GetJobStartInstancePosition(j20));
    EXPECT_LONGS_EQUAL(1, sfOrder.GetJobFinishInstancePosition(j20));
    EXPECT_LONGS_EQUAL(2, sfOrder.GetJobStartInstancePosition(j00));
    EXPECT_LONGS_EQUAL(3, sfOrder.GetJobFinishInstancePosition(j00));
    EXPECT_LONGS_EQUAL(4, sfOrder.GetJobStartInstancePosition(j10));
    EXPECT_LONGS_EQUAL(5, sfOrder.GetJobFinishInstancePosition(j10));
}

TEST(WhetherSkipInsertStart_finish, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    // SFOrder sfOrder(tasksInfo, initial);
    JobCEC j00(0, 0);
    JobCEC j10(1, 0);
    JobCEC j20(2, 0);
    for (uint i = 0; i < 6; i++)
    {
        std::cout << "Test i: " << i << std::endl;
        {
            SFOrder sfOrder(tasksInfo, initial);
            sfOrder.RemoveJob(j00);
            EXPECT(!WhetherSkipInsertStart(j00, i, tasksInfo, sfOrder));
            EXPECT(!WhetherSkipInsertFinish(j00, i, tasksInfo, sfOrder));
        }
        {
            SFOrder sfOrder(tasksInfo, initial);
            sfOrder.RemoveJob(j10);
            EXPECT(!WhetherSkipInsertStart(j10, i, tasksInfo, sfOrder));
            EXPECT(!WhetherSkipInsertFinish(j10, i, tasksInfo, sfOrder));
        }
        {
            SFOrder sfOrder(tasksInfo, initial);
            sfOrder.RemoveJob(j20);
            EXPECT(!WhetherSkipInsertStart(j20, i, tasksInfo, sfOrder));
            EXPECT(!WhetherSkipInsertFinish(j20, i, tasksInfo, sfOrder));
        }
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
    sfOrder.RemoveJob(j00);
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
    VectorDynamic expect = sRes.startTimeVector_;
    expect << 0, 2, 3;
    EXPECT(assert_equal(expect, sRes.startTimeVector_));
}

TEST(sched, v2)
{
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 10, 1, 1;
    SFOrder sfOrder(tasksInfo, initial);
    sfOrder.print();
    std::vector<uint> processorJobVec_;
    processorJobVec_.clear();
    VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder, processorJobVec_);
    EXPECT(assert_equal(initial, initialSTV));
    EXPECT_LONGS_EQUAL(0, processorJobVec_[0]);
    EXPECT_LONGS_EQUAL(0, processorJobVec_[1]);
    EXPECT_LONGS_EQUAL(0, processorJobVec_[2]);
    EXPECT_LONGS_EQUAL(1, processorJobVec_[3]);
    EXPECT(ExamBasic_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec_, processorNum));
}

TEST(SFOrder, insert_erase_v2)
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

    EXPECT_LONGS_EQUAL(0, sfOrder.GetJobStartInstancePosition(j20));
    EXPECT_LONGS_EQUAL(1, sfOrder.GetJobFinishInstancePosition(j20));
    EXPECT_LONGS_EQUAL(2, sfOrder.GetJobStartInstancePosition(j10));
    EXPECT_LONGS_EQUAL(3, sfOrder.GetJobFinishInstancePosition(j10));
    sfOrder.InsertStart(j00, 4);
    sfOrder.InsertFinish(j00, 5);
    sfOrder.print();
    EXPECT_LONGS_EQUAL(0, sfOrder.GetJobStartInstancePosition(j20));
    EXPECT_LONGS_EQUAL(1, sfOrder.GetJobFinishInstancePosition(j20));
    EXPECT_LONGS_EQUAL(2, sfOrder.GetJobStartInstancePosition(j10));
    EXPECT_LONGS_EQUAL(3, sfOrder.GetJobFinishInstancePosition(j10));
    EXPECT_LONGS_EQUAL(4, sfOrder.GetJobStartInstancePosition(j00));
    EXPECT_LONGS_EQUAL(5, sfOrder.GetJobFinishInstancePosition(j00));

    VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder);
    VectorDynamic expect = initialSTV;
    expect << 4, 3, 0;
    EXPECT(assert_equal(expect, initialSTV));
}

TEST(optimize_schedule_when_search_job_order_, v1)
{
    using namespace OrderOptDAG_SPACE;
    NumCauseEffectChain = 1;
    int processorNum = 2;
    considerSensorFusion = 0;
    weightInMpRTDA = 0.5;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    ScheduleResult sRes = ScheduleDAGModel(dagTasks, processorNum);
    PrintSchedule(tasksInfo, sRes.startTimeVector_);
    std::cout << "Obj: " << sRes.obj_ << std::endl;
    EXPECT(sRes.obj_ <= 18);
}
TEST(Schedule, jobOrder)
{
    int processorNum = 1;
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    ScheduleResult res = ScheduleDAGModel(dagTasks, processorNum);
    EXPECT(99 * 2 >= res.obj_);
}

TEST(WhetherSkipInsertStart_finish, v3)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v20.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder(tasksInfo, initial);
    sfOrder.print();
    JobCEC j00(0, 0);
    JobCEC j01(0, 1);
    JobCEC j04(0, 4); // 80-100
    JobCEC j03(0, 3);
    JobCEC j11(1, 1);
    JobCEC j20(2, 0);
    sfOrder.RemoveJob(j04);
    EXPECT(WhetherSkipInsertStart(j04, 0, tasksInfo, sfOrder));
    EXPECT(WhetherSkipInsertStart(j04, 2, tasksInfo, sfOrder));
    EXPECT(!WhetherSkipInsertStart(j04, 4, tasksInfo, sfOrder));
}
TEST(WhetherStartFinishTooLong, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v20.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 1;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initial);
    SFOrder sfOrder(tasksInfo, initial);
    sfOrder.print();
    JobCEC j00(0, 0);
    JobCEC j01(0, 1);
    JobCEC j04(0, 4); // 80-100
    JobCEC j03(0, 3);
    JobCEC j11(1, 1);
    JobCEC j20(2, 0);
    sfOrder.RemoveJob(j00);
    LLint startP = 0;
    sfOrder.InsertStart(j00, startP);
    double accumLengthMin = 0;
    EXPECT(!WhetherStartFinishTooLong(accumLengthMin, j00, 1, tasksInfo, sfOrder, startP));
    EXPECT(!WhetherStartFinishTooLong(accumLengthMin, j00, 2, tasksInfo, sfOrder, startP));
    EXPECT(WhetherStartFinishTooLong(accumLengthMin, j00, 3, tasksInfo, sfOrder, startP));
    std::cout << "After inserting finish at 3: " << std::endl;
    sfOrder.InsertFinish(j00, 3);
    sfOrder.print();
    EXPECT(WhetherStartFinishTooLong(accumLengthMin, j00, 4, tasksInfo, sfOrder, startP));
}

TEST(SFOrder, constructor_v2)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v24.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    std::vector<uint> processorJobVec;
    VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initialSTV);
    SFOrder sfOrder(tasksInfo, initialSTV);
    sfOrder.print();
    VectorDynamic initial2 = SFOrderScheduling(dagTasks, tasksInfo, processorNum, sfOrder);
    PrintSchedule(tasksInfo, initial2);
    EXPECT(gtsam::assert_equal(initialSTV, initial2));
}
TEST(Obj, RTDA_v1)
{
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 10, 1, 1;
    SFOrder sfOrder(tasksInfo, initial);
    sfOrder.print();

    IterationStatus status(dagTasks, tasksInfo, sfOrder, processorNum);
    considerSensorFusion = 0;
    EXPECT_LONGS_EQUAL(18, status.ReadObj());
    EXPECT_DOUBLES_EQUAL(32.5, status.ObjWeighted(), 0.1);
    EXPECT_LONGS_EQUAL(18, status.ObjBarrier());

    considerSensorFusion = 1;
    FreshTol = 0;
    EXPECT_LONGS_EQUAL(18, status.ReadObj());
    EXPECT_LONGS_EQUAL(180, status.ObjBarrier());
    EXPECT_DOUBLES_EQUAL(194.5, status.ObjWeighted(), 0.1);
    FreshTol = 100;
    EXPECT_LONGS_EQUAL(0, status.ObjBarrier());
}
TEST(FindLongestChainJobIndex, v1)
{
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 2;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    initial << 0, 10, 1, 1;
    SFOrder sfOrder(tasksInfo, initial);
    IterationStatus status(dagTasks, tasksInfo, sfOrder, processorNum);
    EXPECT_LONGS_EQUAL(1, FindLongestChainJobIndex(status)[0]);
    initial << 0, 10, 1, 11;
    SFOrder sfOrder2(tasksInfo, initial);
    IterationStatus status2(dagTasks, tasksInfo, sfOrder2, processorNum);
    EXPECT_LONGS_EQUAL(0, FindLongestChainJobIndex(status2)[0]);
}
int main()
{
    TestResult tr;
    // make sure all tests have the correct setting
    NumCauseEffectChain = 1;
    doScheduleOptimization = 0;
    return TestRegistry::runAllTests(tr);
}
