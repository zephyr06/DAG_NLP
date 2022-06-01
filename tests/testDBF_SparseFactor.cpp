#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

using namespace DAG_SPACE;

TEST(testDBF, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddDBF_Factor(graph, tasksInfo);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    std::ofstream os("graph.dot");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    graph.saveGraph(os, initialEstimateFG);
#pragma GCC diagnostic pop

    double expect = 641;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}