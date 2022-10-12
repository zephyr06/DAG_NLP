#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Optimization/OptimizeOrder.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Factors/Interval.h"

using namespace DAG_SPACE;

TEST(constructor, ListSchedulingGivenOrder)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    VectorDynamic actual = ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder);
    assert_equal(initial, actual);
}

TEST(constructor, ListSchedulingGivenOrder_v2)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrderMultiCore jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef, 1);
    PrintSchedule(tasksInfo, initial);

    JobOrderMultiCore jobOrderCurr = jobOrderRef;
    JobCEC job1 = jobOrderCurr[4];
    JobCEC job2 = jobOrderCurr[9];
    jobOrderCurr.ChangeJobStartOrder(4, 9);
    IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr, 1);
    EXPECT(job1 == jobOrderCurr[9]);
    EXPECT(job2 == jobOrderCurr[8]);
    // EXPECT_LONGS_EQUAL(8, GetStartTime(JobCEC{1, 0}, statusCurr.startTimeVector_, tasksInfo));
    // EXPECT_LONGS_EQUAL(7, GetStartTime(JobCEC{3, 0}, statusCurr.startTimeVector_, tasksInfo));
    PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
}

TEST(constructor, JobOrderMultiCore)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    std::vector<JobCEC> jobOrderExpect{JobCEC{5, 0}, JobCEC{0, 0}, JobCEC{1, 0}, JobCEC{5, 1}, JobCEC{0, 1}, JobCEC{2, 0}, JobCEC{3, 0}, JobCEC{5, 2}, JobCEC{0, 2}, JobCEC{4, 0}};
    AssertEqualVectorExact<JobCEC>(jobOrderExpect, jobOrder.jobOrder_);
}

TEST(JobOrder, change_order)
{
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrderMultiCore jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef, 1);

    JobOrderMultiCore jobOrderCurr = jobOrderRef;
    jobOrderCurr.ChangeJobStartOrder(0, 5);
    EXPECT(jobOrderRef[0] == jobOrderCurr[5]);
    EXPECT(jobOrderRef[5] == jobOrderCurr[4]);
    JobCEC j1 = jobOrderRef[0];
    JobCEC j2 = jobOrderRef[5];
    EXPECT_LONGS_EQUAL(5, jobOrderCurr.jobIndexMap_[j1]);
    EXPECT_LONGS_EQUAL(4, jobOrderCurr.jobIndexMap_[j2]);
}

TEST(Schedule_optimize, MakeProgress)
{
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrderMultiCore jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef, 1);
    {
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderRef, 1);
        EXPECT(!MakeProgress(statusPrev, statusCurr));
        std::cout << "Initial schedule" << std::endl;
        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
    }
    {
        JobOrderMultiCore jobOrderCurr = jobOrderRef;
        jobOrderCurr.ChangeJobStartOrder(4, 5);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr, 1);
        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
        EXPECT(MakeProgress(statusPrev, statusCurr));
    }
    {
        JobOrderMultiCore jobOrderCurr = jobOrderRef;
        jobOrderCurr.ChangeJobStartOrder(4, 5);
        jobOrderCurr.ChangeJobStartOrder(16, 17);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr, 1);
        EXPECT(MakeProgress(statusPrev, statusCurr));
    }
}

TEST(Schedule, jobOrder)
{
    coreNumberAva = 1;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    ScheduleResult res = ScheduleDAGModel<LSchedulingKnownTA>(dagTasks);
    EXPECT(99 >= res.rtda_.reactionTime);
    EXPECT(99 >= res.rtda_.dataAge);
    // EXPECT_LONGS_EQUAL(99, res.rtda_.reactionTime);
    // EXPECT_LONGS_EQUAL(99, res.rtda_.dataAge);
}

TEST(list_scheduling, least_finish_time_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v74.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    std::cout << initial << std::endl;
    PrintSchedule(tasksInfo, initial);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
}

TEST(list_scheduling, ListSchedulingGivenOrderPA_v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v74.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // this is probably a little embarssed, consider designing a better way
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);

    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder);
    std::cout << initial << std::endl;
    PrintSchedule(tasksInfo, initial);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
}

TEST(ListSchedulingGivenOrder, strict_job_order)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v74.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // this is probably a little embarssed, consider designing a better way
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);

    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    JobCEC j1 = jobOrder[6];
    JobCEC j2 = jobOrder[9];
    jobOrder.ChangeJobStartOrder(6, 9); // j1 to the end of j2
    EXPECT(jobOrder.jobIndexMap_[j2] < jobOrder.jobIndexMap_[j1]);
    VectorDynamic actual = ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder);
    EXPECT(GetStartTime(j1, actual, tasksInfo) > GetStartTime(j2, actual, tasksInfo));
    PrintSchedule(tasksInfo, actual);
}

TEST(changeOrderNP, insert_delete)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    jobOrder.insertNP(0);
    // jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    JobCEC j0 = jobOrder.jobOrder_[0];
    EXPECT(jobOrder.jobOrderSerial_[0]);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobIndexMap_[j0]);

    jobOrder.insertNP(1);
    // jobOrder.ChangeJobOrderNonParallel(1, 1);
    JobCEC j1 = jobOrder.jobOrder_[1];
    EXPECT(jobOrder.jobOrderSerial_[1]);

    jobOrder.eraseNP(0);
    EXPECT(!jobOrder.jobOrderSerial_[0]);

    jobOrder.eraseNP(j1);
    EXPECT(!jobOrder.jobOrderSerial_[1]);
}

TEST(changeOrderNP, switch_position)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    jobOrder.insertNP(0);
    // jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    JobCEC j0 = jobOrder.jobOrder_[0];
    jobOrder.insertNP(1);
    // jobOrder.ChangeJobOrderNonParallel(1, 1);
    JobCEC j1 = jobOrder.jobOrder_[1];
    jobOrder.ChangeJobOrder(0, 1); // swap j0 and j1

    EXPECT_LONGS_EQUAL(1, jobOrder.jobIndexMap_[j0]);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobIndexMap_[j1]);
    EXPECT(jobOrder.jobOrderSerial_[0]);
    EXPECT(jobOrder.jobOrderSerial_[1]);
}

TEST(changeOrderNP, ListSchedulingLFTPA)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 2,1,0

    jobOrder.ChangeJobStartOrder(0, 1);
    EXPECT_LONGS_EQUAL(1, jobOrder.jobOrder_[0].taskId);
    EXPECT_LONGS_EQUAL(2, jobOrder.jobOrder_[1].taskId);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobOrder_[2].taskId);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder);
    VectorDynamic expected = initial;
    expected << 50, 0, 20;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

TEST(changeOrderNP, ListSchedulingLFTPA_MultiCore)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 1,2,0

    jobOrder.ChangeJobStartOrder(0, 1);
    EXPECT_LONGS_EQUAL(2, jobOrder.jobOrder_[0].taskId);
    EXPECT_LONGS_EQUAL(1, jobOrder.jobOrder_[1].taskId);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobOrder_[2].taskId);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    VectorDynamic expected = initial;
    expected << 20, 0, 0;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

TEST(changeOrderNP, ListSchedulingLFTPA_MultiCore_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 1,2,0

    jobOrder.ChangeJobStartOrder(1, 2);
    EXPECT_LONGS_EQUAL(1, jobOrder.jobOrder_[0].taskId);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobOrder_[1].taskId);
    EXPECT_LONGS_EQUAL(2, jobOrder.jobOrder_[2].taskId);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    VectorDynamic expected = initial;
    expected << 0, 0, 10;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

TEST(changeOrderNP, ListSchedulingLFTPA_MultiCore_v3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 1,2,0

    JobCEC j1 = jobOrder[1];
    JobCEC j4 = jobOrder[4];
    jobOrder.ChangeJobStartOrder(1, 4);
    EXPECT_LONGS_EQUAL(j4.taskId, jobOrder.jobOrder_[3].taskId);
    EXPECT_LONGS_EQUAL(j1.taskId, jobOrder.jobOrder_[4].taskId);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    PrintSchedule(tasksInfo, initial);
    VectorDynamic expected = initial;
    expected << 0, 100, 110, 100, 0;
    EXPECT(assert_equal(expected, initial));
}

TEST(changeOrderNP, ListSchedulingLFTPA_NP)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial); //
    PrintSchedule(tasksInfo, initial);

    // jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    jobOrder.insertNP(0);
    // jobOrder.ChangeJobOrderNonParallel(1, 1);
    jobOrder.insertNP(1);
    jobOrder.ChangeJobOrder(0, 1); // swap j0 and j1

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3, jobOrder);
    VectorDynamic expected = initial;
    expected << 1, 0, 1;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

TEST(changeOrderNP, ListSchedulingLFTPA_NP_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial); //
    PrintSchedule(tasksInfo, initial);

    // jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    jobOrder.insertNP(0);
    // jobOrder.ChangeJobOrderNonParallel(1, 1);
    jobOrder.insertNP(1);
    jobOrder.ChangeJobOrder(0, 1); // swap j0 and j1

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3, jobOrder);
    VectorDynamic expected = initial;
    expected << 20, 100, 0, 100, 20;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

// TEST(ListSchedulingLFTPA_NP, jobOrderConflict)
// {
//     using namespace DAG_SPACE;
//     DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
//     JobOrderMultiCore jobOrder(tasksInfo, initial); //
//     PrintSchedule(tasksInfo, initial);

//     // jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
//     jobOrder.insertNP(0);
//     // jobOrder.ChangeJobOrderNonParallel(1, 1);
//     jobOrder.insertNP(1);
//     jobOrder.ChangeJobOrder(0, 1); // swap j0 and j1

//     initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3, jobOrder);
//     VectorDynamic expected = initial;
//     expected << 0, 0, 0;
//     EXPECT(assert_equal(expected, initial));

//     PrintSchedule(tasksInfo, initial);
// }

TEST(IO, ReadWriteResult)
{
    DAG_SPACE::ScheduleResult res;
    res.schedulable_ = false;
    res.timeTaken_ = 1.1;
    res.obj_ = 2.1;
    std::string dirStr = PROJECT_PATH + "build/";
    const char *pathDataset = (dirStr).c_str();
    std::string file = "testIO.txt";
    WriteToResultFile(pathDataset, file, res, 0);
    DAG_SPACE::ScheduleResult res2 = ReadFromResultFile(pathDataset, file, 0);
    EXPECT_DOUBLES_EQUAL(1.1, res2.timeTaken_, 1e-3);
    EXPECT_DOUBLES_EQUAL(2.1, res2.obj_, 1e-3);
    EXPECT(!res2.schedulable_);
}

TEST(changeOrderNP, ExamPrecedenceJobSatisfied)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 0,1,2
    PrintSchedule(tasksInfo, initial);
    jobOrder.jobOrder_ = {JobCEC{0, 0}, JobCEC{1, 0}, JobCEC{2, 0}};

    jobOrder.insertNP(0);
    jobOrder.insertNP(1);
    JobCEC j0(0, 0);
    JobCEC j1(1, 0);
    JobCEC j2(2, 0);
    std::vector<LLint> jobScheduled(true ? ((jobOrder).size()) : 0, -1);

    LLint timeNow = 0;
    EXPECT((ExamPrecedenceJobSatisfiedNP(j0, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT((!ExamPrecedenceJobSatisfiedNP(j1, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT((ExamPrecedenceJobSatisfiedNP(j2, timeNow, jobOrder, jobScheduled, tasksInfo)));

    EXPECT((ExamPrecedenceJobSatisfied(j0, jobScheduled, jobOrder)));
    EXPECT((!ExamPrecedenceJobSatisfied(j1, jobScheduled, jobOrder)));
    EXPECT((!ExamPrecedenceJobSatisfied(j2, jobScheduled, jobOrder)));

    jobScheduled[0] = 0;
    EXPECT((ExamPrecedenceJobSatisfiedNP(j2, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT(!(ExamPrecedenceJobSatisfied(j2, jobScheduled, jobOrder)));

    timeNow = 2;
    jobScheduled[0] = 0;
    EXPECT((ExamPrecedenceJobSatisfiedNP(j1, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT((ExamPrecedenceJobSatisfiedNP(j2, timeNow, jobOrder, jobScheduled, tasksInfo)));

    EXPECT((ExamPrecedenceJobSatisfied(j1, jobScheduled, jobOrder)));
    EXPECT(!(ExamPrecedenceJobSatisfied(j2, jobScheduled, jobOrder)));

    jobOrder.ChangeJobStartOrder(0, 1); // 1,0,2
    timeNow = 0;
    jobScheduled[0] = -1;

    EXPECT((ExamPrecedenceJobSatisfiedNP(j1, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT((ExamPrecedenceJobSatisfied(j1, jobScheduled, jobOrder)));
    EXPECT((!ExamPrecedenceJobSatisfiedNP(j0, timeNow, jobOrder, jobScheduled, tasksInfo)));
    EXPECT((!ExamPrecedenceJobSatisfied(j0, jobScheduled, jobOrder)));
    EXPECT(!(ExamPrecedenceJobSatisfied(j2, jobScheduled, jobOrder)));
    EXPECT((ExamPrecedenceJobSatisfiedNP(j2, timeNow, jobOrder, jobScheduled, tasksInfo)));
}

TEST(changeOrderNP, ListSchedulingLFTPA_NP_overwrite_P)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    PrintSchedule(tasksInfo, initial);
    jobOrder.jobOrder_ = {JobCEC{0, 0}, JobCEC{2, 0}, JobCEC{1, 0}};

    jobOrder.insertNP(0);
    jobOrder.insertNP(1);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    VectorDynamic expected = initial;
    expected << 0, 2, 2;
    EXPECT(assert_equal(expected, initial));

    // jobOrder.ChangeJobOrder(0, 1);
    // initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    // expected << 0, 0, 0; // P and NP are conflicted
    // EXPECT(assert_equal(expected, initial));

    jobOrder.ChangeJobStartOrder(0, 1);
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder);
    expected << 3, 3, 0; // P and NP are consistent
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

TEST(JobOrderMultiCore, optimize)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    PrintSchedule(tasksInfo, initial);
    VectorDynamic actual = initial;
    actual << 0, 0, 0;
    assert_equal(actual, initial);
    ScheduleResult res = ScheduleDAGModel<LSchedulingFreeTA>(dagTasks, 3);
    PrintSchedule(tasksInfo, res.startTimeVector_);
    EXPECT_LONGS_EQUAL(12, res.obj_);
}

TEST(WhetherSkipSwitch, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v12.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    JobCEC j00(0, 0);
    JobCEC j01(0, 1);
    JobCEC j10(1, 0);
    JobCEC j20(2, 0);
    JobCEC j21(2, 1);
    EXPECT(WhetherSkipSwitch(tasksInfo, j00, j01));
    EXPECT(WhetherSkipSwitch(tasksInfo, j20, j21));
    EXPECT(WhetherSkipSwitch(tasksInfo, j21, j20));
    EXPECT(WhetherSkipSwitch(tasksInfo, j00, j00));
    EXPECT(WhetherSkipSwitch(tasksInfo, j00, j21));
    EXPECT(!WhetherSkipSwitch(tasksInfo, j00, j20));
    EXPECT(!WhetherSkipSwitch(tasksInfo, j01, j20));
}

TEST(ListSchedulingLFTPA, processorIdVec)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    PrintSchedule(tasksInfo, initial);
    jobOrder.jobOrder_ = {JobCEC{0, 0}, JobCEC{2, 0}, JobCEC{1, 0}};

    std::vector<uint> processorJobVec;

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder, processorJobVec);
    VectorDynamic actualAssignment = Vector2Eigen<uint>(processorJobVec);
    VectorDynamic expected = actualAssignment;
    expected << 0, 1, 1;
    EXPECT(assert_equal(expected, actualAssignment));

    jobOrder.ChangeJobStartOrder(0, 1);
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder, processorJobVec);
    PrintSchedule(tasksInfo, initial);
    actualAssignment = Vector2Eigen<uint>(processorJobVec);
    expected << 1, 1, 0;
    EXPECT(assert_equal(expected, actualAssignment));
}

TEST(ListSchedulingLFTPA, processorIdVec_multi_rate)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    std::vector<uint> processorJobVec;

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder, processorJobVec);
    VectorDynamic actualAssignment = Vector2Eigen<uint>(processorJobVec);
    VectorDynamic expected = actualAssignment;
    expected << 0, 0, 1, 1, 0;
    EXPECT(assert_equal(expected, actualAssignment));
}


TEST(ExamFeasibility, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 3;
    std::vector<uint> processorJobVec;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, jobOrder, processorJobVec);
    EXPECT(ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    EXPECT(!ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum - 1));
    EXPECT(!ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum - 2));
}
TEST(ExamFeasibility, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    std::vector<uint> processorJobVec;

    int processorNum = 2;
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, jobOrder, processorJobVec);
    PrintSchedule(tasksInfo, initial);

    EXPECT(ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(0) = 1;
    EXPECT(!ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(0) = 0;
    initial(1) = 99;
    EXPECT(!ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(1) = 100;
    initial(4) = 30;
    EXPECT(ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(4) = 8;
    EXPECT(!ExamFeasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
