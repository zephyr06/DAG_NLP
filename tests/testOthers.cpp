#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/GraphUtilsFromBGL.h"

TEST(AnalyticJaocbian_DBF, v1)
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
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, dagTasks.tasks, sizeOfVariables,
                                errorDimensionDBF, mapIndex,
                                maskForEliminate, processorTaskSet,
                                model);

    // VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 4, -1, 6, 8;
    MatrixDynamic expect = factor.NumericalDerivativeDynamicUpperDBF(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factor.DBFJacobian(factor.f, startTimeVector, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);
}

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

    vector<double> initialUpdateVec;
    initialUpdateVec.reserve(variableDimension - 1);
    LLint indexUpdate = 0;
    for (size_t i = 0; i < variableDimension; i++)
    {
        if (not maskForEliminate[i])
        {
            initialUpdateVec.push_back(initial(i, 0));
        }
    }
    VectorDynamic initialUpdate;
    initialUpdate.resize(initialUpdateVec.size(), 1);
    for (size_t i = 0; i < initialUpdateVec.size(); i++)
    {
        initialUpdate(i, 0) = initialUpdateVec[i];
    }
    MatrixDynamic expect = factorAfter.NumericalDerivativeDynamicUpperDBF(factorAfter.f, initialUpdate, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factorAfter.DBFJacobian(factorAfter.f, initial, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);

    int a = 1;
}

TEST(elimination_secret, v2)
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
    auto h_before = factor.JacobianAnalytic(initial);
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
    auto h_after = factorAfter.JacobianAnalytic(initial);
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