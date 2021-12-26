#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

using namespace DAG_SPACE;


TEST(testDDL, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddDDL_Factor(graph, tasksInfo);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    double expect = 0;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 207, 5, 3, 104, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 25;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 97, 5, 3, 104, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 97, 5, 3, 204, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 13;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}