#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

TEST(Overlap, v1)
{
    // coreNumberAva = 1;
    Interval v1(0, 10), v2(11, 10);
    AssertEqualScalar(0, Overlap(v1, v2));

    v1.start = 100;
    AssertEqualScalar(0, Overlap(v1, v2));
    v1.start = 0;

    v2.start = -5;
    AssertEqualScalar(5, Overlap(v1, v2));
    v2.start = 11;

    v2.start = 1;
    v2.length = 5;
    AssertEqualScalar(5, Overlap(v1, v2));

    v2.start = -1;
    v2.length = 100;
    AssertEqualScalar(10, Overlap(v1, v2));

    v2.start = 5;
    v2.length = 10;
    AssertEqualScalar(5, Overlap(v1, v2));
}

TEST(testDBF, v1)
{
    // coreNumberAva = 1;
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    VectorDynamic compressed;
    compressed.resize(2, 1);
    compressed << 0, 180;
    vector<bool> maskForEliminate{false, false, 0, 0, false};
    MAP_Index2Data mapIndex;
    mapIndex[0] = MappingDataStruct{0, 0};
    mapIndex[1] = MappingDataStruct{1, 0};
    mapIndex[2] = MappingDataStruct{2, 0};
    mapIndex[3] = MappingDataStruct{3, 0};
    mapIndex[4] = MappingDataStruct{4, 0};
    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 2, 3, 4, 5;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 90;
    VectorDynamic dbfExpect1 = factor.f(startTimeVector);
    auto dbfActual = DbfIntervalOverlapError(startTimeVector, 0,
                                             processorTaskSet, tasks, tasksInfo.sizeOfVariables);
    // VectorDynamic actual = OrderOptDAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    AssertEqualScalar(dbfExpect1(0, 0), dbfActual);
    assert_equal(dbfExpect, dbfExpect1);

    startTimeVector << 1, 2, 3, 4, 19;
    dbfActual = DbfIntervalOverlapError(startTimeVector, 0,
                                        processorTaskSet, tasks, tasksInfo.sizeOfVariables);
    AssertEqualScalar(54, dbfActual);
}
TEST(testDBF, v2)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < tasksInfo.variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 128;

    double dbfActual = DbfIntervalOverlapError(startTimeVector, 0,
                                               processorTaskSet, tasks, tasksInfo.sizeOfVariables);
    VectorDynamic dbfActual2 = factor.f(startTimeVector);
    AssertEqualScalar(128, dbfActual);
    assert_equal(dbfExpect, dbfActual2);
}
TEST(testDBF, v3MultiProcess)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v29.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    VectorDynamic dbfExpect;
    dbfExpect.resize(2, 1);
    dbfExpect << 68, 32;
    double dbfActual0 = DbfIntervalOverlapError(startTimeVector, 0,
                                                processorTaskSet, tasks, sizeOfVariables);
    double dbfActual1 = DbfIntervalOverlapError(startTimeVector, 1,
                                                processorTaskSet, tasks, sizeOfVariables);

    VectorDynamic dbfActual2 = factor.f(startTimeVector);
    AssertEqualScalar(dbfExpect(0, 0), dbfActual0);
    AssertEqualScalar(dbfExpect(1, 0), dbfActual1);
    assert_equal(dbfExpect, dbfActual2);
}
TEST(testDBF, v4MultiProcess)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v30.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    VectorDynamic dbfExpect;
    dbfExpect.resize(5, 1);
    dbfExpect << 9, 0, 13, 11, 0;
    VectorDynamic dbfActual2 = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual2);
}
TEST(testDBF, v5MultiProcess)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v31.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    VectorDynamic dbfExpect;
    dbfExpect.resize(5, 1);
    dbfExpect << 0, 0, 0, 0, 0;
    VectorDynamic dbfActual2 = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual2);
}
// TODO: not done yet
TEST(NumericalDerivativeDynamicUpperDBF, v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v22.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    // declare variables
    vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 30.1636, 129.772, 40.7072, 0.008139, 105, 27.9779, 100, 3.072, 103.393;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    MatrixDynamic jacob_actual = factor.NumericalDerivativeDynamicUpperDBF(factor.f, initialSTV, deltaOptimizer, errorDimensionDBF);
}

TEST(NumericalDerivativeDynamicUpperDBF, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    MatrixDynamic jacob_actual = factor.NumericalDerivativeDynamicUpperDBF(factor.f, initialSTV, deltaOptimizer, errorDimensionDBF);
    MatrixDynamic jacob_expect;
    jacob_expect.resize(1, 8);
    // if all the vanishing gradient is considered:
    //  jacob_expect << 1.5, -2, -4, -1, 0.5, 0, 3.5, 1.5;
    jacob_expect << 1.5, -2, -4, -1, 0.5, 0.5, 3, 1.5;
    AssertEigenEqualMatrix(jacob_expect, jacob_actual);
}

TEST(AnalyticJaocbian_DBF, v1)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    forestInfo.AddLinearEliminate(1, 3, 5);
    forestInfo.AddLinearEliminate(3, 5, 5);
    forestInfo.AddLinearEliminate(2, 4, 5);

    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 4, -1, 6, 8;
    MatrixDynamic expect = factor.NumericalDerivativeDynamicUpperDBF(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factor.DBFJacobian(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);
}

TEST(AnalyticJaocbian_DBF, v2)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    forestInfo.AddLinearEliminate(1, 3, 5);
    forestInfo.AddLinearEliminate(3, 5, 5);
    forestInfo.AddLinearEliminate(2, 4, 5);

    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 2, 3, 4, 5;
    MatrixDynamic expect = factor.NumericalDerivativeDynamicUpperDBF(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factor.DBFJacobian(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);
    // assert_equal(expect, actual);
}

TEST(AnalyticJaocbian_DBF, v3)
{
    using namespace OrderOptDAG_SPACE;
    auto dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v36.csv", "orig");
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);

    MatrixDynamic expect = factor.NumericalDerivativeDynamicUpperDBF(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factor.DBFJacobian(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}