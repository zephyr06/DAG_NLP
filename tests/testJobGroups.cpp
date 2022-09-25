#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/Optimize.h"

using namespace DAG_SPACE;
TEST(FindJobIndexWithError, v1)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);

    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 46, 92.4;
    auto actual = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    EXPECT_LONGS_EQUAL(2, actual.size());
    // PrintKeyVector(actual[0]);
    // PrintKeyVector(actual[1]);
    EXPECT_LONGS_EQUAL(0, AnalyzeKey(actual[0][0]).first);
    EXPECT_LONGS_EQUAL(5, AnalyzeKey(actual[0][1]).first);
    EXPECT_LONGS_EQUAL(1, AnalyzeKey(actual[1][0]).first);
    EXPECT_LONGS_EQUAL(5, AnalyzeKey(actual[1][1]).first);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}