#include "../sources/Optimize.h"
TEST(random, v1)
{
    // VectorDynamic a;
    // a.resize(6, 1);
    // for (int i = 0; i < 6; i++)
    // {
    //     a(i, 0) = i;
    // }
    // cout << a << endl;
    ;
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