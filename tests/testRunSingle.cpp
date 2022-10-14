#include "sources/Optimize.h"
#include "sources/testMy.h"
#include "sources/GraphUtilsFromBGL.h"
#include "sources/profilier.h"
TEST(DAG_Optimize_schedule, v1)
{
    using namespace RTSS21IC_NLP;
    BeginTimer("main");
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    auto sth = OptimizeScheduling(tasks);

    VectorDynamic res = sth.optimizeVariable;

    cout << "The result after optimization is " << Color::green << sth.optimizeError
         //  << Color::blue << res
         << Color::def << endl;
    EndTimer("main");
    PrintTimer();
}
// TEST(SensorFusionDiscrete, v1)
// {
//     BeginTimer("main");
//     using namespace DAG_SPACE;
//     // DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH+"TaskData/" + testDataSetName + ".csv", "orig");

//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH+"TaskData/test_n5_v38.csv", "orig");

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