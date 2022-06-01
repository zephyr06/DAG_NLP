#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

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
    expect = 257;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 97, 5, 3, 104, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 117;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 97, 5, 3, 204, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 245;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}
TEST(ddl, v2)
{
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v52.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    EliminationForest forestInfo(tasksInfo);

    VectorDynamic initialEstimate = GenerateInitialForDAG_IndexMode(dagTasks,
                                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 13, 30, 80, 93;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    NonlinearFactorGraph graph;
    AddDDL_Factor(graph, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    double expect = 24.5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}