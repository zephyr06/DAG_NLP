#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(MakeSpanFactor_analytic, v1)
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
    double makeSpanDef = makespanWeight;
    Symbol key('a', 0);
    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    MakeSpanFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionMS,
                          model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    SM_Dynamic actual = factor.JacobianAnalytic(startTimeVector);
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionMS);
    assert_equal(expect, actual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 201;
    actual = factor.JacobianAnalytic(startTimeVector);
    expect = NumericalDerivativeDynamicUpper(factor.f, startTimeVector, deltaOptimizer, errorDimensionMS);
    assert_equal(expect, actual);
    makespanWeight = makeSpanDef;
}

TEST(FindFinishTime, v1)
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
    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    MakeSpanFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionMS,
                          model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic finishTimeActual = factor.FindFinishTime(startTimeVector);
    VectorDynamic expect = startTimeVector;
    expect << 16, 117, 16, 15, 116, 15, 14, 115;
    assert_equal(expect, finishTimeActual);
}
TEST(FindFinishTime, v2)
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
    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    forestInfo.AddLinearEliminate(1, 3, 5);
    forestInfo.AddLinearEliminate(3, 5, 5);
    forestInfo.AddLinearEliminate(2, 4, 5);

    MakeSpanFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionMS,
                          model);

    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 6, 104, 2, 0, 101;
    VectorDynamic finishTimeActual = factor.FindFinishTime(startTimeVector);
    VectorDynamic expect = finishTimeActual;
    expect << 16, 116, 15, 14, 115;
    assert_equal(expect, finishTimeActual);
}
TEST(FindLargeSmall, v1)
{
    VectorDynamic x;
    x.resize(10, 1);
    x << 2.01, 5.6, 6, 4, 8,
        1.5, 3, 4, 8, 6;
    LLint le, ls, se, ss;
    FindLargeSmall(x, x, se, ss, le, ls);
    AssertEqualScalar(5, se);
    AssertEqualScalar(0, ss);
    AssertEqualScalar(8, le);
    AssertEqualScalar(4, ls);
}
TEST(FindLargeSmall, v2)
{
    VectorDynamic x;
    x.resize(10, 1);
    x << 2, 5, 6, 4, 10,
        1, 3, 4, 8, 6;
    LLint le, ls, se, ss;
    FindLargeSmall(x, x, se, ss, le, ls);
    AssertEqualScalar(5, se);
    AssertEqualScalar(0, ss);
    AssertEqualScalar(4, le);
    AssertEqualScalar(8, ls);
}

TEST(testMakeSpan, v1)
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
    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    MakeSpanFactor factor(key, dagTasks, tasksInfo, forestInfo, errorDimensionMS,
                          model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic msExpect = GenerateVectorDynamic(errorDimensionMS);
    msExpect << 117;
    double makeSpanDef = makespanWeight;
    makespanWeight = 1;
    VectorDynamic actual = factor.f(startTimeVector);
    assert_equal(msExpect, actual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 201;
    msExpect(0, 0) = 215;
    actual = factor.f(startTimeVector);
    assert_equal(msExpect, actual);
    makespanWeight = makeSpanDef;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}