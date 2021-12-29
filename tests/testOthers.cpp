#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/GraphUtilsFromBGL.h"
TEST(elimination_secret, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v53.csv", "orig");
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
    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    };
    vector<bool> maskForEliminate(variableDimension, false);
    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    initial << 174.454, 309.782, 716.536, 1132.16, 1698.74, 153.668, 399.858, 799.917, 1199.83, 1599.66, 120.021, 548.711, 1014.5, 1503.89, 4.07109, 200, 407.673, 600.044, 807.673, 1002.69, 1208.21, 1400, 1608.21, 1800, 77.5402, 503.698, 937.437, 1199.92, 1598.73;
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    Symbol key('b', 0);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, processorTaskSet, model);
    auto h_before = factor.DBFJacobian(factor.f, initial, deltaOptimizer, errorDimensionDBF);
    bool whetherEliminate = false;
    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(dagTasks);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;
    // this function performs in-place modification for all the variables!
    // TODO: should we add eliminate function for sensorFusion?
    factor.addMappingFunction(initial, mapIndex, whetherEliminate, maskForEliminate,
                              eliminationTrees, indexesBGL);

    DBF_ConstraintFactor factorAfter(key, tasks, sizeOfVariables, errorDimensionDBF,
                                     mapIndex, maskForEliminate, processorTaskSet, model);
    auto h_after = factorAfter.DBFJacobian(factorAfter.f, initial, deltaOptimizer, errorDimensionDBF);
    // h_before.resize(8, 7);
    // h_after.resize(8, 7);
    cout << "H before: " << endl
         << h_before << endl;
    cout << "H after: " << endl
         << h_after << endl;

    int a = 1;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}