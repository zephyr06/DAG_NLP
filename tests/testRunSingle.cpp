#include "sources/Optimization/Optimize.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Tools/profilier.h"
#include "sources/Tools/testMy.h"
TEST(DAG_Optimize_schedule, v1)
{
    BeginTimer("main");
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/" + testDataSetName + ".csv", "orig");

    for (size_t i = 100; i < 200; i++)
    {
        auto sth = OptimizeScheduling(tasks, i);

        VectorDynamic res = sth.optimizeVariable;

        cout << "The result after optimization is " << Color::green << sth.optimizeError
             << Color::def << endl;
        if (sth.optimizeError < 1e-3)
        {
            std::cout << "Total re-seed times: " << i - 100 << std::endl;
            break;
        }
        if (PrintOutput)
            cout << Color::blue << res << Color::def << endl;
    }
    EndTimer("main");
    PrintTimer();
}
// TEST(SensorFusionDiscrete, v1)
// {
//     BeginTimer("main");
//     using namespace DAG_SPACE;
//     // DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/" + testDataSetName + ".csv", "orig");

//     auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v38.csv", "orig");

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