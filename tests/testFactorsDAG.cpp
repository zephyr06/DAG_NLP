#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(testDAG, v1)
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
    LLint errorDimensionDAG = dagTasks.edgeNumber();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    DAG_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionDAG,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic dbfExpect = GenerateVectorDynamic(errorDimensionDAG);
    dbfExpect << 8, 9, 10, 10, 9, 10;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 20, 101;

    dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(96, dbfActual.sum());
}

TEST(AnalyticJaocbian_DAG, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;

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
    LLint errorDimensionDAG = dagTasks.edgeNumber();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DAG_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionDAG,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDAG);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
    AssertEigenEqualMatrix(expect, actual);
}
TEST(AnalyticJaocbian_DAG, v2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;

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

    mapIndex[1] = MappingDataStruct{3, 5};
    mapIndex[3] = MappingDataStruct{5, 5};
    mapIndex[2] = MappingDataStruct{4, 5};
    maskForEliminate[1] = true;
    maskForEliminate[2] = true;
    maskForEliminate[3] = true;

    Symbol key('a', 0);
    LLint errorDimensionDAG = dagTasks.edgeNumber();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    forestInfo.AddLinearEliminate(1, 3, 5);
    forestInfo.AddLinearEliminate(3, 5, 5);
    forestInfo.AddLinearEliminate(2, 4, 5);

    DAG_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionDAG,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 4, -1, 6, 8;
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDAG);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
}
TEST(AnalyticJaocbian_DAG, v3)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v36.csv", "orig");
    TaskSet tasks = dagTasks.tasks;

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
    LLint errorDimensionDAG = dagTasks.edgeNumber();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DAG_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionDAG,
                                model);

    VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);

    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDAG);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}