#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

using namespace DAG_SPACE;

// class DBF_Factor : public InequalityFactor2D
// {
// public:
//     TaskSetInfoDerived tasksInfo;
//     LLint index1;
//     LLint index2;

// };

TEST(testDAG, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
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

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}