#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(Prior_factor, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
    vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }

    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    LLint errorDimensionPrior = 1;
    vector<int> order = FindDependencyOrder(dagTasks);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);

    Prior_ConstraintFactor factor(key, tasksInfo, forestInfo,
                                  errorDimensionPrior, 0.0, order[0], model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    AssertEqualScalar(0, factor.f(startTimeVector)(0, 0));
    startTimeVector << 6, 107, 5, 3, 104, 2, 10, 101;
    AssertEqualScalar(10 * weightPrior_factor, factor.f(startTimeVector)(0, 0));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}