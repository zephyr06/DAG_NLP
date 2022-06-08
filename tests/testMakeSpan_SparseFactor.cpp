#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/MakeSpanFactor.h"
#include "sources/Factors/MultiKeyFactor.h"

TEST(GenerateKeysMS, start)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto keyS = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 's');
    auto keyE = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 'e');
    vector<int> b = {2, 3, 4};

    vector<LLint> c = {0, 0, 0};
    std::vector<gtsam::Symbol> exp1 = GenerateKey(b, c);
    AssertBool(true, exp1 == keyS, __LINE__);
    vector<int> a = {0};
    vector<LLint> d = {1};
    std::vector<gtsam::Symbol> exp2 = GenerateKey(a, d);
    AssertBool(true, exp2 == keyE, __LINE__);
}
TEST(GenerateKeysMS, start2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto keyS = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 's');
    auto keyE = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 'e');
    vector<int> b = {0, 1, 2, 3, 4};

    vector<LLint> c = {0, 0, 0, 0, 0};
    std::vector<gtsam::Symbol> exp1 = GenerateKey(b, c);
    AssertBool(true, exp1 == keyS, __LINE__);
    vector<int> a = {0, 1, 2, 3, 4};
    vector<LLint> d = {3, 1, 1, 0, 0};
    std::vector<gtsam::Symbol> exp2 = GenerateKey(a, d);
    AssertBool(true, exp2 == keyE, __LINE__);
}

TEST(TESTms, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
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
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
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
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v3.csv", "orig");
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

TEST(TESTms, v_Preempt)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev, true);

    VectorDynamic startTimeVector;
    startTimeVector.resize(10, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo, true);
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