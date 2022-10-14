#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Baseline/RTSS21IC.h"
TEST(DAG_Optimize_schedule, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");
    auto sth = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, 1000, 1000);
    EXPECT(sth.schedulable_);
    sth = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, 60, 60);
    EXPECT(!sth.schedulable_);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
