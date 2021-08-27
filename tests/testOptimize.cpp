#include "../sources/Optimize.h"
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
    VectorDynamic actual = DAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    if (expected != actual)
    {
        cout << red << "RecoverStartTimeVector failed!" << def << endl;
        throw;
    }
}

// TEST(sensorFusion, v1)
// {
//     using namespace DAG_SPACE;
//     using namespace RegularTaskSystem;

//     TaskSet tasks = ReadTaskSet("../TaskData/test_n5_v1.csv", "orig");
//     int N = tasks.size();
//     LLint hyperPeriod = HyperPeriod(tasks);

//     // declare variables
//     vector<LLint> sizeOfVariables;
//     int variableDimension = 0;
//     for (int i = 0; i < N; i++)
//     {

//         LLint size = hyperPeriod / tasks[i].period;
//         sizeOfVariables.push_back(size);
//         variableDimension += size;
//     }
//     LLint errorDimensionSF = 1;
//     Symbol key('a', 0);
//     auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
//     SensorFusion_ConstraintFactor factor(key, tasks, sizeOfVariables,
//                                          errorDimensionSF, sensorFusionTolerance, model);
//     VectorDynamic initial = GenerateInitialForDAG(tasks, sizeOfVariables, variableDimension);
//     // initial << 0, 1, 2, 3, 4;
//     auto sth = factor.evaluateError(initial);
//     cout << sth << endl;
// }

TEST(BigIndex2TaskIndex, v1)
{
    using namespace DAG_SPACE;
    TaskSet tasks = ReadTaskSet("../TaskData/test_n5_v3.csv", "orig");
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

// TEST(testDBF, v1)
// {
//     using namespace DAG_SPACE;
//     TaskSet tasks = ReadTaskSet("../TaskData/test_n5_v1.csv", "orig");
//     int N = tasks.size();
//     LLint hyperPeriod = HyperPeriod(tasks);

//     // declare variables
//     vector<LLint> sizeOfVariables;
//     int variableDimension = 0;
//     for (int i = 0; i < N; i++)
//     {
//         LLint size = hyperPeriod / tasks[i].period;
//         sizeOfVariables.push_back(size);
//         variableDimension += size;
//     }

//     VectorDynamic compressed;
//     compressed.resize(2, 1);
//     compressed << 0, 180;
//     vector<bool> maskForEliminate{false, false, 0, 0, false};
//     MAP_Index2Data mapIndex;
//     mapIndex[0] = MappingDataStruct{0, 0};
//     mapIndex[1] = MappingDataStruct{1, 0};
//     mapIndex[2] = MappingDataStruct{2, 0};
//     mapIndex[3] = MappingDataStruct{3, 0};
//     mapIndex[4] = MappingDataStruct{4, 0};
//     Symbol key('a', 0);
//     LLint errorDimensionDBF = 1;
//     auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
//     DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
//                                 mapIndex, maskForEliminate, model);
//     VectorDynamic startTimeVector;
//     startTimeVector.resize(5, 1);
//     startTimeVector << 1, 2, 3, 4, 5;
//     VectorDynamic dbfExpect = factor.f(startTimeVector);
//     VectorDynamic dbfActual = factor.DbfInterval(startTimeVector);
//     // VectorDynamic actual = DAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
//     if (dbfExpect != dbfActual)
//     {
//         cout << red << "testDBF failed!" << def << endl;
//         throw;
//     }
// }

TEST(ExtractVariable, v1)
{
    using namespace DAG_SPACE;
    TaskSet tasks = ReadTaskSet("../TaskData/" + testDataSetName + ".csv", "orig");
    auto res = OptimizeScheduling(tasks);
    cout << "The result after optimization is " << res << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}