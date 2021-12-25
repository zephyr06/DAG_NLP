#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

using namespace DAG_SPACE;
void AddDDL_Factor(NonlinearFactorGraph &graph, DAG_Model &dagTasks)
{

    LLint errorDimensionDDL = 1;
    model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);

    LLint index = 0;
    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
            Symbol key1(FACTOR2KEY["DDL"], index++);
            graph.emplace_shared<InequalifyFactor1D>(key1, tasksInfo, forestInfo,
                                                     errorDimensionDDL, model);
            Symbol key2(FACTOR2KEY["DDL"], index++);
            res(indexRes++, 0) = Barrier(tasksInfo.tasks[i].deadline + j * tasksInfo.tasks[i].period -
                                         ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                         tasksInfo.tasks[i].executionTime + 0) *
                                 weightDDL_factor;
            // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
            res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                         (j * tasksInfo.tasks[i].period) + 0) *
                                 weightDDL_factor;
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

    Symbol key('a', 0);
    LLint errorDimensionDDL = 2 * variableDimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    DDL_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDDL,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic dbfExpect = GenerateVectorDynamic(2 * variableDimension);
    VectorDynamic dbfActual = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 201;
    dbfExpect(14, 0) = 15 * weightDDL_factor;
    dbfActual = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}