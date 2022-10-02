#include <CppUnitLite/TestHarness.h>

#include "sources/Optimization/JobOrder.h"

using namespace DAG_SPACE;
TEST(constructor, JobOrder)
{

    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    JobOrder jobOrder(tasksInfo, initial);
    VectorDynamic actual = ListSchedulingGivenOrder(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, jobOrder);
    assert_equal(initial, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
