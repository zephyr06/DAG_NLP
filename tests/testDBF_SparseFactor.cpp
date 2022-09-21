#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/PreemptConstraintFactor.h"
#include "sources/Factors/DBF_ConstraintFactorPreemptive.h"

using namespace DAG_SPACE;

TEST(testDBF, v1)
{
    whetherRandomNoiseModelSigma = 0;
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
TEST(preemptFactor, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddPreempt_Factor(graph, tasksInfo, true);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG;
    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            Symbol key = GenerateKey(i, j);
            VectorDynamic v;
            v = GenerateVectorDynamic(2);
            v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j),
                ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);
            initialEstimateFG.insert(key, v);
        }
    }

    double actual = graph.error(initialEstimateFG);
    double expect = (10 * 10 * 2 + 11 * 11 + 12 * 12 * 2 + 13 * 13 + 14 * 14 * 2) / 2.0;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

TEST(FindPossibleOverlapKeys, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    auto actual = FindPossibleOverlapKeys(0, 0, 1, 0, tasksInfo);
    EXPECT_LONGS_EQUAL(8, actual.size());
}

// TEST(dbf_preempt, graph_jacobian)
// {
//     using namespace DAG_SPACE;
//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     EliminationForest forestInfo(tasksInfo);
//     VectorDynamic startTimeVector;
//     startTimeVector.resize(8, 1);
//     startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
//     Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo, true);

//     NonlinearFactorGraph graph;
//     AddDBFPreempt_Factor(graph, tasksInfo);

//     std::cout << Color::green;
//     auto sth = graph.linearize(initialEstimateFG)->jacobian();
//     MatrixDynamic jacobianCurr = sth.first;
//     std::cout << "Current Jacobian matrix:" << std::endl;
//     std::cout << jacobianCurr << std::endl;
//     std::cout << "Current b vector: " << std::endl;
//     std::cout << sth.second << std::endl;
//     std::cout << Color::def << std::endl;
// }

// TEST(dbf_preempt, graph_error)
// {
//     using namespace DAG_SPACE;
//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     EliminationForest forestInfo(tasksInfo);
//     VectorDynamic startTimeVector;
//     startTimeVector.resize(8, 1);
//     startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
//     Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo, true);

//     NonlinearFactorGraph graph;
//     AddDBFPreempt_Factor(graph, tasksInfo);
//     double actual = graph.error(initialEstimateFG);
//     std::vector<double> expected = {0, -10, -20, -32, -44,
//                                     0, -10, -20, -32, -44,
//                                     0, 0, 0, -12, -24,
//                                     0, 0, 0, -12, -24,
//                                     0, 0, 0, 0, 0,
//                                     0, -9, -20,
//                                     0, 0, -11};
//     EXPECT_DOUBLES_EQUAL(SquareError(expected), actual, 1e-6);
// }

// TEST(testDBF, Preemptv1)
// {
//     using namespace DAG_SPACE;
//     auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     EliminationForest forestInfo(tasksInfo);

//     VectorDynamic startTimeVector;
//     startTimeVector.resize(8, 1);
//     startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
//     Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo, true);

//     std::vector<gtsam::Symbol> keys = FindPossibleOverlapKeys(0, 0, 1, 0, tasksInfo);
//     double actual = DBF_PreemptError(initialEstimateFG, keys, tasksInfo)(0);
//     EXPECT_DOUBLES_EQUAL(10, actual, 1e-6);

//     keys = FindPossibleOverlapKeys(1, 0, 0, 0, tasksInfo);
//     actual = DBF_PreemptError(initialEstimateFG, keys, tasksInfo)(0);
//     EXPECT_DOUBLES_EQUAL(10, actual, 1e-6);

//     keys = FindPossibleOverlapKeys(4, 0, 0, 0, tasksInfo);
//     actual = DBF_PreemptError(initialEstimateFG, keys, tasksInfo)(0);
//     EXPECT_DOUBLES_EQUAL(44, actual, 1e-6);

//     keys = FindPossibleOverlapKeys(4, 1, 0, 1, tasksInfo);
//     actual = DBF_PreemptError(initialEstimateFG, keys, tasksInfo)(0);
//     EXPECT_DOUBLES_EQUAL(20, actual, 1e-6);
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}