#include "sources/Optimization/Optimize.h"
#include "sources/Baseline/RTSS21IC.h"
#include "sources/Tools/profilier.h"
#include "sources/Tools/testMy.h"
TEST(DAG_Optimize_schedule, v1)
{
    BeginTimer("main");
    using namespace OrderOptDAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    auto sth = ScheduleRTSS21IC(tasks, sensorFusionTolerance, FreshTol);
    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}