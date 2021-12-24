#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(testDDL, v1)
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
TEST(GenerateInitialForDAG_RM_DAG, vDDL)
{
    using namespace DAG_SPACE;
    // this task is not schedulable even if only DAG and schedulability constraints are considered
    DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v16.csv", "orig");
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
    LLint errorDimensionDDL = 2 * variableDimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    DDL_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDDL,
                                model);
    InitializeMethod sth = initializeMethod;
    initializeMethod = RM;
    VectorDynamic startTimeVector = GenerateInitial(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic dbfExpect = GenerateVectorDynamic(2 * variableDimension);
    VectorDynamic dbfActual = factor.f(startTimeVector);
    // assert_equal(dbfExpect, dbfActual);
    initializeMethod = sth;
}

TEST(AnalyticJaocbian_DDL, NumericalDerivativeDynamicUpperv4)

{
    // cout << "AnalyticJaocbian_DDL, v4" << endl;
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
    LLint errorDimensionDDL = 2 * variableDimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DDL_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDDL,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDDL);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);

    // add mapping
    mapIndex[1] = MappingDataStruct{3, 5};
    forestInfo.AddLinearEliminate(1, 3, 5);

    expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDDL);
    actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
    AssertEigenEqualMatrix(expect, actual);
}
TEST(AnalyticJaocbian_DDL, v2)
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
    LLint errorDimensionDDL = 2 * variableDimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    forestInfo.AddLinearEliminate(1, 3, 5);
    forestInfo.AddLinearEliminate(3, 5, 5);
    forestInfo.AddLinearEliminate(2, 4, 5);
    DDL_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDDL,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 4, -1, 6, 8;
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDDL);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
}

TEST(AnalyticJaocbian_DDL, v3)
{
    // cout << "AnalyticJaocbian_DDL, v3" << endl;
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
    LLint errorDimensionDDL = 2 * variableDimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DDL_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDDL,
                                model);

    VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionDDL);
    MatrixDynamic actual = factor.JacobianAnalytic(startTimeVector);
    assert_equal(expect, actual);
    AssertEigenEqualMatrix(expect, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}