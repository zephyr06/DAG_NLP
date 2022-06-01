#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

TEST(sensorFusion, v1)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, tasksInfo.sizeOfVariables);
    AssertEqualScalar(1, errorDimensionSF);
    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < tasksInfo.variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initial;
    initial.resize(5, 1);
    initial << 2, 1, 0, 3, 4;
    auto sthh = withAddedSensorFusionError;
    withAddedSensorFusionError = 1;
    double defaultSF = sensorFusionTolerance;
    sensorFusionTolerance = 2;
    auto sth = factor.evaluateError(initial);
    AssertEqualScalar(27, sth(0, 0)); // 9 before

    // cout << sth << endl;
    initial << 3, 5, 1, 6, 7;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(25, sth(0, 0)); // 8 before

    sensorFusionTolerance = defaultSF;
    withAddedSensorFusionError = sthh;
}

TEST(sensorFusion, v2)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;
    auto sthh = withAddedSensorFusionError;
    withAddedSensorFusionError = 1;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    double freshtolCurr = FreshTol;

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

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
    AssertEqualScalar(3, errorDimensionSF);
    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initialEstimate = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    // cout << initialEstimate << endl;

    VectorDynamic initial;
    initial.resize(8, 1);
    initial << 6, 107, 5, 3, 104, 2, 0, 101;

    double defaultSF = sensorFusionTolerance;
    sensorFusionTolerance = 2;
    FreshTol = 10000;
    auto sth = factor.evaluateError(initial);

    AssertEqualScalar(27, sth(0, 0)); // Node 0-1
    AssertEqualScalar(0, sth(1, 0));  // Node 0-2
    AssertEqualScalar(29, sth(2, 0)); // Node 1
    sensorFusionTolerance = 0;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(29, sth(0, 0)); // Node 0
    AssertEqualScalar(2, sth(1, 0));  // Node 0
    AssertEqualScalar(30, sth(2, 0)); // Node 1

    //     initial << 6, 107, 5, 3, 104, 2, 0, 101;
    // sth = factor.evaluateError(initial);
    // AssertEqualScalar(7, sth(0, 0));
    initial << 16, 107, 5, 3, 104, 2, 0, 101;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(2, sth(0, 0));  // Node 0
    AssertEqualScalar(2, sth(1, 0));  // Node 0
    AssertEqualScalar(30, sth(2, 0)); // Node 1
    sensorFusionTolerance = defaultSF;
    FreshTol = freshtolCurr;
    withAddedSensorFusionError = sthh;
}

TEST(sensorFusion, v3)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v38.csv", "orig");
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

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);

    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    // bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initial;
    initial.resize(6, 1);
    initial << 50.097, 39.065, 14.026, 26.054, 0.0137, 100;
    sensorFusionTolerance = 39;
    auto sth = factor.evaluateError(initial);
    double defaultSF = sensorFusionTolerance;

    AssertEqualScalar(0, sth(0, 0));

    sensorFusionTolerance = defaultSF;
}

TEST(sensorFusion_AnalyticJacobian, v1)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;
    auto sthh = withAddedSensorFusionError;
    withAddedSensorFusionError = 1;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
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

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
    AssertEqualScalar(3, errorDimensionSF);
    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initialEstimate = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);

    VectorDynamic initial;
    initial.resize(8, 1);
    initial << 6, 107, 5, 3, 104, 2, 0, 101;

    double defaultSF = sensorFusionTolerance;
    sensorFusionTolerance = 2;
    auto expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    auto actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);
    sensorFusionTolerance = 0;
    expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);

    initial << 6, 107, 5, 3, 104, 2, 0, 101;
    expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);
    initial << 16, 107, 5, 3, 104, 2, 0, 101;
    expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);
    sensorFusionTolerance = defaultSF;
    withAddedSensorFusionError = sthh;
}
TEST(sensorFusion_AnalyticJacobian, v2)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;
    double defaultSF = sensorFusionTolerance;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v38.csv", "orig");
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

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);

    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    // bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initial;
    initial.resize(6, 1);
    initial << 50.097, 39.065, 14.026, 26.054, 0.0137, 100;
    sensorFusionTolerance = 39;
    auto expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    auto actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);

    sensorFusionTolerance = defaultSF;
}
TEST(sensorFusion_AnalyticJacobian, v3)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;
    double defaultSF = sensorFusionTolerance;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v9.csv", "orig");
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

    Symbol key('a', 0);
    LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
    AssertEqualScalar(1, errorDimensionSF);
    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    // bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo, forestInfo,
                                         errorDimensionSF, sensorFusionTolerance,
                                         model);
    VectorDynamic initial;
    initial.resize(5, 1);
    initial << 2, 1, 0, 3, 4;
    auto sthh = withAddedSensorFusionError;
    withAddedSensorFusionError = 1;
    sensorFusionTolerance = 2;
    auto expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    auto actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);

    // cout << sth << endl;
    initial << 3, 5, 1, 6, 7;
    expect = NumericalDerivativeDynamicUpper(factor.f, initial, deltaOptimizer, errorDimensionSF);
    actual = factor.JacobianAnalytic(initial);
    assert_equal(expect, actual);

    sensorFusionTolerance = defaultSF;
    withAddedSensorFusionError = sthh;
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}