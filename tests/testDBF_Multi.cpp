#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

TEST(testDBF, multi_core)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v46.csv", "orig");
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

    VectorDynamic compressed;
    compressed.resize(5, 1);
    compressed << 43, 31, 19, 18, 17;
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
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor_Multi factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                      model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 43, 31, 19, 18, 17;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 12;
    coreNumberAva = 5;
    VectorDynamic actual = factor.f(startTimeVector);
    assert_equal(dbfExpect, actual);
}

TEST(testDBF, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v46.csv", "orig");
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
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor_Multi factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                      model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 70, 50, 19, 18, 17;
    coreNumberAva = 5;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 12;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(dbfExpect(0, 0), dbfActual(0, 0));
}

TEST(testDBF, v2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v46.csv", "orig");
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
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor_Multi factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                      model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 70, 50, 19, 18, 17;
    coreNumberAva = 3;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 37;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(dbfExpect(0, 0), dbfActual(0, 0));
}
TEST(testDBF, v3)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v47.csv", "orig");
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
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor_Multi factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                      model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 3, 5, 6, 9;
    coreNumberAva = 3;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 12;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(dbfExpect(0, 0), dbfActual(0, 0));
    coreNumberAva = 4;
    dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(5, dbfActual(0, 0));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}