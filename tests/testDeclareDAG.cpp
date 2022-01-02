#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(RecoverStartTimeVector, v1)
{
    VectorDynamic compressed;
    compressed.resize(2, 1);
    compressed << 0, 180;
    vector<bool> maskEliminate{false, true, true, true, false};
    MAP_Index2Data mapIndex;
    mapIndex[0] = MappingDataStruct{0, 0};
    mapIndex[1] = MappingDataStruct{0, 10};
    mapIndex[2] = MappingDataStruct{1, 10};
    mapIndex[3] = MappingDataStruct{2, 10};
    mapIndex[4] = MappingDataStruct{4, 0};
    VectorDynamic expected;
    expected.resize(5, 1);
    expected << 0, 10, 20, 30, 180;
    VectorDynamic actual = RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    if (expected != actual)
    {
        cout << Color::red << "RecoverStartTimeVector failed!" << Color::def << endl;
        throw;
    }
}

TEST(RecoverStartTime, v2)
{
    VectorDynamic compressed;
    compressed.resize(2, 1);
    compressed << 0, 100;
    vector<bool> maskEliminate;
    maskEliminate = {1, 0, 1, 1, 0, 1, 1};
    MAP_Index2Data mapIndex;
    MappingDataStruct t0(LLint(1), 11.0);
    MappingDataStruct t1(LLint(1), 0.0);
    MappingDataStruct t2(LLint(4), 15.0);
    MappingDataStruct t3(LLint(0), 10.0);
    MappingDataStruct t4(LLint(4), 0.0);
    MappingDataStruct t5(LLint(0), 22.0);
    MappingDataStruct t6(LLint(0), 35.0);
    mapIndex[0] = t0;
    mapIndex[1] = t1;
    mapIndex[2] = t2;
    mapIndex[3] = t3;
    mapIndex[4] = t4;
    mapIndex[5] = t5;
    mapIndex[6] = t6;
    VectorDynamic actual = RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    VectorDynamic expect;
    expect.resize(7, 1);
    expect << 11, 0, 115, 21, 100, 33, 46;

    assert_equal(expect, actual);
}
TEST(RecoverStartTimeVector, v3)
{
    VectorDynamic compressed;
    compressed.resize(2, 1);
    compressed << 0, 180;
    vector<bool> maskEliminate{false, true, true, true, false};
    MAP_Index2Data mapIndex;
    mapIndex[0] = MappingDataStruct{0, 0};
    mapIndex[1] = MappingDataStruct{2, 10};
    mapIndex[2] = MappingDataStruct{3, 10};
    mapIndex[3] = MappingDataStruct{4, 10};
    mapIndex[4] = MappingDataStruct{4, 0};
    VectorDynamic expected;
    expected.resize(5, 1);
    expected << 0, 210, 200, 190, 180;
    VectorDynamic actual = RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    assert_equal(expected, actual);
}

TEST(ExtractVariable, v1)
{
    using namespace DAG_SPACE;
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
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 0, 0));
    AssertEqualScalar(107, ExtractVariable(initialSTV, sizeOfVariables, 0, 1));
    AssertEqualScalar(5, ExtractVariable(initialSTV, sizeOfVariables, 1, 0));
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 2, 0));
    AssertEqualScalar(104, ExtractVariable(initialSTV, sizeOfVariables, 2, 1));
    AssertEqualScalar(2, ExtractVariable(initialSTV, sizeOfVariables, 3, 0));
    AssertEqualScalar(0, ExtractVariable(initialSTV, sizeOfVariables, 4, 0));
    AssertEqualScalar(102, ExtractVariable(initialSTV, sizeOfVariables, 4, 1));
}

TEST(ExtractVariable, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v20.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104;
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 0, 0));
    AssertEqualScalar(107, ExtractVariable(initialSTV, sizeOfVariables, 1, 0));
    AssertEqualScalar(5, ExtractVariable(initialSTV, sizeOfVariables, 2, 0));
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 3, 0));
    AssertEqualScalar(104, ExtractVariable(initialSTV, sizeOfVariables, 4, 0));
}

TEST(BigIndex2TaskIndex, v1)
{
    using namespace DAG_SPACE;
    TaskSet tasks = ReadTaskSet("../../TaskData/test_n5_v3.csv", "orig");
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

    if (0 != BigIndex2TaskIndex(0, sizeOfVariables) ||
        1 != BigIndex2TaskIndex(5, sizeOfVariables) ||
        2 != BigIndex2TaskIndex(6, sizeOfVariables) ||
        3 != BigIndex2TaskIndex(8, sizeOfVariables) ||
        4 != BigIndex2TaskIndex(9, sizeOfVariables))
    {
        cout << "Error in BigIndex2TaskIndex" << endl;
        throw;
    }
}

TEST(CompresStartTimeVector, v1)
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
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    VectorDynamic actual = CompresStartTimeVector(startTimeVector, maskForEliminate);
    VectorDynamic expect;
    expect.resize(5, 1);
    expect << 1, 5, 6, 7, 8;
    AssertEigenEqualVector(expect, actual);
}

TEST(NumericalDerivativeDynamic2D1, v1)
{
    NormalErrorFunction2D f = [](VectorDynamic x1, VectorDynamic x2)
    {
        return x1 + x2;
    };
    VectorDynamic x1 = GenerateVectorDynamic(3);
    x1 << 1, 2, 3;
    VectorDynamic x2 = GenerateVectorDynamic(3);
    x2 << 4, 5, 6;
    MatrixDynamic actual1 = NumericalDerivativeDynamic2D1(f, x1, x2, deltaOptimizer, 3);
    MatrixDynamic expect1 = GenerateMatrixDynamic(3, 3);
    expect1 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    MatrixDynamic actual2 = NumericalDerivativeDynamic2D2(f, x1, x2, deltaOptimizer, 3);
    assert_equal(expect1, actual1);
    assert_equal(expect1, actual2);
}

TEST(NumericalDerivativeDynamic2D1, v2)
{
    NormalErrorFunction2D f = [](VectorDynamic x1, VectorDynamic x2)
    {
        VectorDynamic res = x1;
        res(0, 0) = x1(0, 0) * x2(0, 0);
        res(1, 0) = x1(1, 0) + x2(0, 0);
        res(2, 0) = x1(2, 0) + x2(0, 0) * x2(0, 0);
        return res;
    };
    VectorDynamic x1 = GenerateVectorDynamic(3);
    x1 << 1, 2, 3;
    VectorDynamic x2 = GenerateVectorDynamic(3);
    x2 << 4, 5, 6;
    MatrixDynamic actual1 = NumericalDerivativeDynamic2D1(f, x1, x2, deltaOptimizer, 3);
    MatrixDynamic expect1 = GenerateMatrixDynamic(3, 3);
    expect1 << 4, 0, 0,
        0, 1, 0,
        0, 0, 1;
    MatrixDynamic expect2 = GenerateMatrixDynamic(3, 3);
    expect2 << 1, 0, 0,
        1, 0, 0,
        8, 0, 0;
    MatrixDynamic actual2 = NumericalDerivativeDynamic2D2(f, x1, x2, deltaOptimizer, 3);
    assert_equal(expect1, actual1);
    assert_equal(expect2, actual2);
}
TEST(AnalyzeKey, v1)
{
    auto key1 = GenerateKey(0, 0);
    auto p1 = AnalyzeKey(key1);
    AssertEqualScalar(0, p1.first);
    AssertEqualScalar(0, p1.second);
}
TEST(AnalyzeKey, v2)
{
    auto key1 = GenerateKey(9, 9);
    auto p1 = AnalyzeKey(key1);
    AssertEqualScalar(9, p1.first);
    AssertEqualScalar(9, p1.second);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}