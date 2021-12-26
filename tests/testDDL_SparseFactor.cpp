#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

using namespace DAG_SPACE;
void AddDDL_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
{

    LLint errorDimensionDDL = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    TaskSet &tasks = tasksInfo.tasks;

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            Symbol key = GenerateKey(index_overall);

            // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
            graph.emplace_shared<SmallerThanFactor1D>(key, tasks[i].deadline + j * tasks[i].period, weightDDL_factor, model);
            // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
            graph.emplace_shared<LargerThanFactor1D>(key, j * tasks[i].period, weightDDL_factor, model);
        }
    }
}

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