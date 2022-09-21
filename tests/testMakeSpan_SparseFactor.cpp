#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/MakeSpanFactor.h"
#include "sources/Factors/MultiKeyFactor.h"

TEST(TESTms, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    // initialEstimateFG.print();
    double actual = graph.error(initialEstimateFG);
    double expect = 6844.5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}
TEST(TESTms, v2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    // initialEstimateFG.print();
    double actual = graph.error(initialEstimateFG);
    double expect = 32;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);

    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 201;
    initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    actual = graph.error(initialEstimateFG);
    expect = 6844.5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

TEST(TESTms, v3)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);

    VectorDynamic startTimeVector;
    startTimeVector.resize(10, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    // initialEstimateFG.print();
    double actual = graph.error(initialEstimateFG);
    double expect = 264.5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}