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

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    JobOrder jobOrder(tasksInfo, initial);
    VectorDynamic actual = ListSchedulingGivenOrder(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, jobOrder);
    assert_equal(initial, actual);
}

TEST(constructor, JobOrder)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    JobOrder jobOrder(tasksInfo, initial);

    std::vector<JobCEC> jobOrderExpect{JobCEC{5, 0}, JobCEC{0, 0}, JobCEC{1, 0}, JobCEC{5, 1}, JobCEC{0, 1}, JobCEC{2, 0}, JobCEC{3, 0}, JobCEC{5, 2}, JobCEC{0, 2}, JobCEC{4, 0}};
    AssertEqualVectorExact<JobCEC>(jobOrderExpect, jobOrder.jobOrder_);
}

TEST(Schedule, jobOrder)
{
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    ScheduleResult res = ScheduleDAGModel(dagTasks);
    EXPECT_LONGS_EQUAL(99, res.rtda_.reactionTime);
    EXPECT_LONGS_EQUAL(99, res.rtda_.dataAge);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
