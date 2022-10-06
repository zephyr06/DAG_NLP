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

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
    JobOrder jobOrder(tasksInfo, initial);
    VectorDynamic actual = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder);
    assert_equal(initial, actual);
}

TEST(constructor, ListSchedulingGivenOrder_v2)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrder jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef);
    PrintSchedule(tasksInfo, initial);

    JobOrder jobOrderCurr = jobOrderRef;
    JobCEC job1 = jobOrderCurr[4];
    JobCEC job2 = jobOrderCurr[9];
    jobOrderCurr.ChangeJobOrder(4, 9);
    IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr);
    EXPECT(job1 == jobOrderCurr[9]);
    EXPECT(job2 == jobOrderCurr[8]);
    // EXPECT_LONGS_EQUAL(8, GetStartTime(JobCEC{1, 0}, statusCurr.startTimeVector_, tasksInfo));
    // EXPECT_LONGS_EQUAL(7, GetStartTime(JobCEC{3, 0}, statusCurr.startTimeVector_, tasksInfo));
    PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
}

TEST(constructor, JobOrder)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
    JobOrder jobOrder(tasksInfo, initial);

    std::vector<JobCEC> jobOrderExpect{JobCEC{5, 0}, JobCEC{0, 0}, JobCEC{1, 0}, JobCEC{5, 1}, JobCEC{0, 1}, JobCEC{2, 0}, JobCEC{3, 0}, JobCEC{5, 2}, JobCEC{0, 2}, JobCEC{4, 0}};
    AssertEqualVectorExact<JobCEC>(jobOrderExpect, jobOrder.jobOrder_);
}

TEST(JobOrder, change_order)
{
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrder jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef);

    JobOrder jobOrderCurr = jobOrderRef;
    jobOrderCurr.ChangeJobOrder(0, 5);
    EXPECT(jobOrderRef[0] == jobOrderCurr[5]);
    EXPECT(jobOrderRef[5] == jobOrderCurr[4]);
    JobCEC j1 = jobOrderRef[0];
    JobCEC j2 = jobOrderRef[5];
    EXPECT_LONGS_EQUAL(5, jobOrderCurr.jobOrderMap_[j1]);
    EXPECT_LONGS_EQUAL(4, jobOrderCurr.jobOrderMap_[j2]);
}

TEST(Schedule_optimize, MakeProgress)
{
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n10_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
    EXPECT(SchedulabilityCheck(dagTasks, tasksInfo, initial));
    JobOrder jobOrderRef(tasksInfo, initial);
    IterationStatus<LSchedulingKnownTA> statusPrev(dagTasks, tasksInfo, jobOrderRef);
    {
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderRef);
        EXPECT(!MakeProgress(statusPrev, statusCurr));
        std::cout << "Initial schedule" << std::endl;
        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
    }
    {
        JobOrder jobOrderCurr = jobOrderRef;
        jobOrderCurr.ChangeJobOrder(4, 5);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr);
        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
        EXPECT(MakeProgress(statusPrev, statusCurr));
    }
    {
        JobOrder jobOrderCurr = jobOrderRef;
        jobOrderCurr.ChangeJobOrder(4, 5);
        jobOrderCurr.ChangeJobOrder(16, 17);
        IterationStatus<LSchedulingKnownTA> statusCurr(dagTasks, tasksInfo, jobOrderCurr);
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
    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);
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
    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);

    PrintSchedule(tasksInfo, initial);
    JobOrder jobOrder(tasksInfo, initial);
    initial = ListSchedulingGivenOrderPA(dagTasks, tasksInfo, jobOrder);
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
    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo, 0);

    PrintSchedule(tasksInfo, initial);
    JobOrder jobOrder(tasksInfo, initial);
    JobCEC j1(3, 1);
    JobCEC j2(1, 1);
    jobOrder.ChangeJobOrder(6, 9); // j1 to the end of j2
    EXPECT(jobOrder.jobOrderMap_[j2] < jobOrder.jobOrderMap_[j1]);
    VectorDynamic actual = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder);
    EXPECT(GetStartTime(j1, actual, tasksInfo) > GetStartTime(j2, actual, tasksInfo));
    PrintSchedule(tasksInfo, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
