#include "sources/Optimization/OptimizeOrder.h"
#include "sources/Tools/profilier.h"
#include "sources/Tools/testMy.h"
TEST(DAG_Optimize_schedule, v1)
{
    BeginTimer("main");
    using namespace DAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    ScheduleResult sth;
    if (processorAssignmentMode == 0)
        sth = DAG_SPACE::ScheduleDAGModel<LSchedulingKnownTA>(dagTasks);
    else if (processorAssignmentMode == 1)
        sth = DAG_SPACE::ScheduleDAGModel<LSchedulingFreeTA>(dagTasks);
    PrintResultAnalyzation(sth, dagTasks);
    EndTimer("main");
    PrintTimer();
}
// TEST(SensorFusionDiscrete, v1)
// {
//     BeginTimer("main");
//     using namespace DAG_SPACE;
//     // DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v38.csv", "orig");

//     VectorDynamic initial;
//     initial.resize(6, 1);
//     // initial<<0,13,25,39,50,60;
//     initial << 0, 0, 0, 0, 0, 1;
//     // sensorFusionTolerance = 100;
//     auto sth = OptimizeScheduling(dagTasks, initial);

//     VectorDynamic res = sth.optimizeVariable;
//     cout << "The result after optimization is " << Color::green << sth.optimizeError
//          << Color::blue << res << Color::def << endl;
//     if (sth.optimizeError > 0.1)
//         CoutError("Not schedulable in one test by NLP");
//     EndTimer("main");
//     PrintTimer();
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}