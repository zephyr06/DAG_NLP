#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

using namespace OrderOptDAG_SPACE;

TEST(testDAG, v1)
{
    whetherRandomNoiseModelSigma = 0;
    weightDAG_factor = 1;
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddDAG_Factor(graph, dagTasks, tasksInfo);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    double expect = 263;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

// TEST(testDAG, preempt_v1)
// {
//     using namespace OrderOptDAG_SPACE;
//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     EliminationForest forestInfo(tasksInfo);

//     NonlinearFactorGraph graph;
//     AddDAG_Factor(graph, dagTasks, tasksInfo, true);

//     VectorDynamic startTimeVector;
//     startTimeVector.resize(8, 1);
//     startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
//     Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo, true);
//     double actual = graph.error(initialEstimateFG);
//     double expect = 263;
//     AssertEqualScalar(expect, actual, 1e-6, __LINE__);
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}