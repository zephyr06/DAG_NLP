#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Optimization/OptimizeOrder.h"

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
    jobOrderCurr.ChangeJobOrder(4, 9);
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
    jobOrderCurr.ChangeJobOrder(0, 5);
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
        jobOrderCurr.ChangeJobOrder(4, 5);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr, 1);
        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
        EXPECT(MakeProgress(statusPrev, statusCurr));
    }
    {
        JobOrderMultiCore jobOrderCurr = jobOrderRef;
        jobOrderCurr.ChangeJobOrder(4, 5);
        jobOrderCurr.ChangeJobOrder(16, 17);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr, 1);
        EXPECT(MakeProgress(statusPrev, statusCurr));
    }
}

TEST(Schedule, jobOrder)
{
    coreNumberAva = 1;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    ScheduleResult res = ScheduleDAGModel<LSchedulingKnownTA>(dagTasks);
    EXPECT_LONGS_EQUAL(99, res.rtda_.reactionTime);
    EXPECT_LONGS_EQUAL(99, res.rtda_.dataAge);
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
    jobOrder.ChangeJobOrder(6, 9); // j1 to the end of j2
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

    jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    EXPECT_LONGS_EQUAL(1, jobOrder.sizeNP());
    JobCEC j0 = jobOrder.jobOrder_[0];
    EXPECT(j0 == jobOrder.jobOrderNonParall_[0]);
    EXPECT_LONGS_EQUAL(0, jobOrder.jobIndexMapNP_[j0]);

    jobOrder.ChangeJobOrderNonParallel(1, 1);
    EXPECT_LONGS_EQUAL(2, jobOrder.sizeNP());
    JobCEC j1 = jobOrder.jobOrder_[1];
    EXPECT(j1 == jobOrder.jobOrderNonParall_[1]);
    EXPECT_LONGS_EQUAL(1, jobOrder.jobIndexMapNP_[j1]);

    jobOrder.ChangeJobOrderNonParallel(0, -1); // delete j0
    EXPECT_LONGS_EQUAL(1, jobOrder.sizeNP());
    EXPECT(j1 == jobOrder.jobOrderNonParall_[0]);

    jobOrder.ChangeJobOrderNonParallel(0, -1); // delete j1
    EXPECT_LONGS_EQUAL(0, jobOrder.sizeNP());
}

TEST(changeOrderNP, switch_position)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    JobCEC j0 = jobOrder.jobOrder_[0];
    jobOrder.ChangeJobOrderNonParallel(1, 1);
    JobCEC j1 = jobOrder.jobOrder_[1];
    jobOrder.ChangeJobOrderNonParallel(0, 1); // swap j0 and j1

    EXPECT_LONGS_EQUAL(2, jobOrder.sizeNP());
    EXPECT(j0 == jobOrder.jobOrderNonParall_[1]);
    EXPECT(j1 == jobOrder.jobOrderNonParall_[0]);
}

TEST(changeOrderNP, ListSchedulingLFTPA)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    JobOrderMultiCore jobOrder(tasksInfo, initial); // 2,1,0

    jobOrder.ChangeJobOrder(0, 1);
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

    jobOrder.ChangeJobOrder(0, 1);
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

    jobOrder.ChangeJobOrder(1, 2);
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
    jobOrder.ChangeJobOrder(1, 4);
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

    jobOrder.ChangeJobOrderNonParallel(0, 0); // insert j0
    JobCEC j0 = jobOrder.jobOrder_[0];
    jobOrder.ChangeJobOrderNonParallel(1, 1);
    JobCEC j1 = jobOrder.jobOrder_[1];
    jobOrder.ChangeJobOrderNonParallel(0, 1); // swap j0 and j1

    jobOrder.ChangeJobOrder(0, 1);

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3, jobOrder);
    VectorDynamic expected = initial;
    expected << 1, 0, 1;
    EXPECT(assert_equal(expected, initial));

    PrintSchedule(tasksInfo, initial);
}

// todo: ADD more tests on ListSchedulingLFTPA

// TEST(JobOrderMultiCore, optimize)
// {
//     using namespace DAG_SPACE;
//     DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v7.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);

//     VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 3);
//     PrintSchedule(tasksInfo, initial);
//     VectorDynamic actual = initial;
//     actual << 0, 0, 0;
//     assert_equal(actual, initial);
//     ScheduleResult res = ScheduleDAGModel<LSchedulingFreeTA>(dagTasks, 3);
//     PrintSchedule(tasksInfo, res.startTimeVector_);
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
