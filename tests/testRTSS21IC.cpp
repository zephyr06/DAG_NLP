#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/OptimizeWithElimination.h"
TEST(DAG_Optimize_schedule, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(RTSS21IC_NLP::PROJECT_PATH + "TaskData/" + RTSS21IC_NLP::testDataSetName + ".csv", "orig");
    // OrderOptDAG_SPACE::ScheduleResult res = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks);
    auto sth = RTSS21IC_NLP::DAG_SPACE::OptimizeScheduling(dagTasks);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
