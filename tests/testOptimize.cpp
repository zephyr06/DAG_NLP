#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/GraphUtilsFromBGL.h"
/*

*/
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
        cout << Color::red << "RecoverStartTimeVector failed!" << Color::def << endl;
        throw;
    }
}

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

TEST(Overlap, v1)
{
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
    VectorDynamic actual = DAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    VectorDynamic expect;
    expect.resize(7, 1);
    expect << 11, 0, 115, 21, 100, 33, 46;

    assert_equal(expect, actual);
}

TEST(FindDependencyOrder, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v16.csv", "orig");
    auto actual = FindDependencyOrder(dagTasks);
    vector<int> expected = {4, 3, 2, 1, 0};
    AssertEqualVectorNoRepeat(expected, actual);
}

TEST(GenerateInitialForDAG, V2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto actual = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 6, 107, 5, 3, 104, 2, 0, 101;
    assert_equal(expected, actual);
}
TEST(GenerateInitialForDAG, V3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v22.csv", "orig");
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
    auto actual = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(9, 1);
    expected << 7, 108, 6, 4, 105, 2, 103, 0, 101;
    assert_equal(expected, actual);
}

TEST(sensorFusion, v1)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v9.csv", "orig");
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
    bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    SensorFusion_ConstraintFactor factor(key, dagTasks, sizeOfVariables,
                                         errorDimensionSF, sensorFusionTolerance,
                                         mapIndex, maskForEliminate,
                                         model);
    VectorDynamic initial;
    initial.resize(5, 1);
    initial << 2, 1, 0, 3, 4;

    double defaultSF = sensorFusionTolerance;
    sensorFusionTolerance = 2;
    auto sth = factor.evaluateError(initial);
    AssertEqualScalar(9, sth(0, 0));

    // cout << sth << endl;
    initial << 3, 5, 1, 6, 7;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(8, sth(0, 0));

    sensorFusionTolerance = defaultSF;
}

TEST(sensorFusion, v2)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    SensorFusion_ConstraintFactor factor(key, dagTasks, sizeOfVariables,
                                         errorDimensionSF, sensorFusionTolerance,
                                         mapIndex, maskForEliminate,
                                         model);
    VectorDynamic initialEstimate = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    // cout << initialEstimate << endl;

    VectorDynamic initial;
    initial.resize(8, 1);
    initial << 6, 107, 5, 3, 104, 2, 0, 101;

    double defaultSF = sensorFusionTolerance;
    sensorFusionTolerance = 2;
    auto sth = factor.evaluateError(initial);
    AssertEqualScalar(8, sth(0, 0)); // Node 0-1
    AssertEqualScalar(0, sth(1, 0)); // Node 0-2
    AssertEqualScalar(9, sth(2, 0)); // Node 1
    sensorFusionTolerance = 0;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(10, sth(0, 0)); // Node 0
    AssertEqualScalar(2, sth(1, 0));  // Node 0
    AssertEqualScalar(10, sth(2, 0)); // Node 1

    //     initial << 6, 107, 5, 3, 104, 2, 0, 101;
    // sth = factor.evaluateError(initial);
    // AssertEqualScalar(7, sth(0, 0));
    initial << 16, 107, 5, 3, 104, 2, 0, 101;
    sth = factor.evaluateError(initial);
    AssertEqualScalar(2, sth(0, 0));  // Node 0
    AssertEqualScalar(2, sth(1, 0));  // Node 0
    AssertEqualScalar(10, sth(2, 0)); // Node 1
    sensorFusionTolerance = defaultSF;
}

TEST(testDBF, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v1.csv", "orig");
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
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(5, 1);
    startTimeVector << 1, 2, 3, 4, 5;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 90;
    VectorDynamic dbfExpect1 = factor.f(startTimeVector);
    VectorDynamic dbfActual = factor.DbfIntervalOverlapError(startTimeVector);
    // VectorDynamic actual = DAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    assert_equal(dbfExpect1, dbfActual);
    assert_equal(dbfExpect, dbfExpect1);

    startTimeVector << 1, 2, 3, 4, 19;
    dbfExpect1(0, 0) = 54;
    dbfActual = factor.DbfIntervalOverlapError(startTimeVector);
    assert_equal(dbfExpect1, dbfActual);
}
TEST(testDBF, v2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic dbfExpect;
    dbfExpect.resize(1, 1);
    dbfExpect << 128;

    VectorDynamic dbfActual = factor.DbfIntervalOverlapError(startTimeVector);
    VectorDynamic dbfActual2 = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual);
    assert_equal(dbfExpect, dbfActual2);
}

TEST(testDDL, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    DDL_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDDL,
                                mapIndex, maskForEliminate, model);
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

TEST(testMakeSpan, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    MakeSpanFactor factor(key, dagTasks, sizeOfVariables, errorDimensionMS,
                          mapIndex, maskForEliminate, model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic msExpect = GenerateVectorDynamic(errorDimensionMS);
    msExpect << 117;
    double makeSpanDef = makespanWeight;
    makespanWeight = 1;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    assert_equal(msExpect, dbfActual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 201;
    msExpect(0, 0) = 117;
    dbfActual = factor.f(startTimeVector);
    assert_equal(msExpect, dbfActual);
    makespanWeight = makeSpanDef;
}

TEST(testDAG, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    DAG_ConstraintFactor factor(key, dagTasks, sizeOfVariables, errorDimensionDAG,
                                mapIndex, maskForEliminate, model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    VectorDynamic dbfExpect = GenerateVectorDynamic(errorDimensionDAG);
    dbfExpect << 8, 9, 10, 10, 9, 10;
    VectorDynamic dbfActual = factor.f(startTimeVector);
    assert_equal(dbfExpect, dbfActual);

    startTimeVector << 6, 107, 5, 3, 104, 2, 20, 101;

    dbfActual = factor.f(startTimeVector);
    AssertEqualScalar(96, dbfActual.sum());
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
    VectorDynamic actual = DAG_SPACE::RecoverStartTimeVector(compressed, maskEliminate, mapIndex);
    assert_equal(expected, actual);
}

TEST(EliminationTree, build_maintain_tree)
{

    // elimination steps:
    // In main loop
    // 1. build a tree with only nodes, no edges

    // 3. maintain the tree structure during loops
    // In 'add eliminate mapping'
    // 3. identify the two considered nodes, and their associated trees
    // 4. extract all the nodes in the two trees, check interval overlap
    // 5. return whether there is overlap or not
    // 6. if yes, cannot add edge or eliminate
    //    if no, then add an edge, and perform eliminate

    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
    auto sth = EstablishGraphStartTimeVector(dagTasks);
    Graph g = sth.first;

    vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    graph_traits<Graph>::vertex_iterator i, end;
    LLint index = 0;
    for (boost::tie(i, end) = vertices(g); i != end; ++i)
    {
        // std::cout << vertex2indexBig[*i] << std::endl;
        AssertEqualScalar(index++, vertex2indexBig[*i]);
    }
}

TEST(EliminationTree, find_sub_tree)
{

    // elimination steps:
    // In main loop
    // 1. build a tree with only nodes, no edges

    // 3. maintain the tree structure during loops
    // In 'add eliminate mapping'
    // 3. identify the two considered nodes, and their associated trees
    // 4. extract all the nodes in the two trees, check interval overlap
    // 5. return whether there is overlap or not
    // 6. if yes, cannot add edge or eliminate
    //    if no, then add an edge, and perform eliminate

    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
    auto sth = EstablishGraphStartTimeVector(dagTasks);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    edge_name_map_t edge2Distance = get(edge_name, g);

    // insert an edge
    for (int i = 1; i < 5; i++)
    {
        Vertex u = indexesBGL[i];
        Vertex v = indexesBGL[i + 1];
        graph_traits<Graph>::edge_descriptor e;
        bool inserted;
        boost::tie(e, inserted) = add_edge(u, v, g);
        if (inserted)
            edge2Distance[e] = 10;
    }

    vector<LLint> res;
    Vertex u = indexesBGL[1];
    FindSubTree(g, res, u);
    AssertEqualVectorNoRepeat({1, 2, 3, 4, 5}, res);

    u = indexesBGL[3];
    res.clear();
    FindSubTree(g, res, u);
    AssertEqualVectorNoRepeat({3, 2, 1, 4, 5}, res);
}

TEST(CheckEliminationTreeConflict, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);

    vector<LLint> tree1 = {2, 3, 5, 0, 1};
    vector<LLint> tree2 = {6, 7};

    vector<bool> maskForEliminate(initialSTV.size(), false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    Symbol key('a', 0);
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);
    AssertBool(false, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
    tree1 = {2, 4, 5, 6};
    tree2 = {3, 1, 0, 7};
    AssertBool(false, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
    tree1 = {2};
    tree2 = {3};
    AssertBool(false, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
    initialSTV << 54, 129, 42, 29, 116, 15, 0, 101;
    AssertBool(true, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
    tree1 = {2, 4, 5, 6};
    tree2 = {3, 1, 0, 7};
    AssertBool(true, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
    initialSTV << 54, 129, 42, 29, 116, 10, 0, 101;
    AssertBool(false, factor.CheckNoConflictionTree(tree1, tree2, initialSTV));
}

TEST(find_sub_tree, v2)
{

    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v18.csv", "orig");
    auto sth = EstablishGraphStartTimeVector(dagTasks);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    edge_name_map_t edge2Distance = get(edge_name, g);

    // add_edge(indexesBGL[0], indexesBGL[5], g);
    // add_edge(indexesBGL[5], indexesBGL[10], g);
    // add_edge(indexesBGL[10], indexesBGL[12], g);
    // add_edge(indexesBGL[1], indexesBGL[6], g);
    add_edge(indexesBGL[2], indexesBGL[7], g);
    add_edge(indexesBGL[19], indexesBGL[7], g);

    vector<LLint> res;
    Vertex u = indexesBGL[19];
    FindSubTree(g, res, u);
    AssertEqualVectorNoRepeat({2, 7, 19}, res);
}

TEST(find_sub_tree, v3)
{

    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v18.csv", "orig");
    auto sth = EstablishGraphStartTimeVector(dagTasks);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    edge_name_map_t edge2Distance = get(edge_name, g);

    // add_edge(indexesBGL[0], indexesBGL[5], g);
    // add_edge(indexesBGL[5], indexesBGL[10], g);
    // add_edge(indexesBGL[10], indexesBGL[12], g);
    // add_edge(indexesBGL[1], indexesBGL[6], g);
    add_edge(indexesBGL[2], indexesBGL[7], g);
    add_edge(indexesBGL[19], indexesBGL[7], g);
    add_edge(indexesBGL[7], indexesBGL[8], g);
    add_edge(indexesBGL[7], indexesBGL[9], g);
    add_edge(indexesBGL[9], indexesBGL[10], g);
    add_edge(indexesBGL[1], indexesBGL[19], g);
    add_edge(indexesBGL[3], indexesBGL[2], g);

    vector<LLint> res;
    Vertex u = indexesBGL[19];
    FindSubTree(g, res, u);
    AssertEqualVectorNoRepeat({2, 3, 8, 7, 19, 9, 1, 10}, res);
}

TEST(FindLeaf, V1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    for (int i = 0; i < N; i++)
        AssertEqualScalar(i, FindLeaf(i, mapIndex));
    mapIndex[1] = MappingDataStruct{2, 1};
    mapIndex[2] = MappingDataStruct{4, 1};
    mapIndex[5] = MappingDataStruct{4, 1};
    AssertEqualScalar(4, FindLeaf(1, mapIndex));
    AssertEqualScalar(4, FindLeaf(5, mapIndex));
}

TEST(FindVanishIndex, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);

    vector<LLint> actual = factor.FindVanishIndex(initialSTV);
    vector<LLint> expected = {0, 5, 6};
    AssertEqualVectorNoRepeat(expected, actual);
}

TEST(FindVanishIndex, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateVectorDynamic(variableDimension - 1);
    initialSTV << 3, 107, 5, 3, 104, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    maskForEliminate[5] = true;
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    mapIndex[5] = MappingDataStruct{2, -3};

    Symbol key('a', 0);
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);

    vector<LLint> actual = factor.FindVanishIndex(initialSTV);
    vector<LLint> expected = {0, 2, 5};
    AssertEqualVectorNoRepeat(expected, actual);
}

TEST(NumericalDerivativeDynamicUpperDBF, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);

    MatrixDynamic jacob_actual = factor.NumericalDerivativeDynamicUpperDBF(factor.f, initialSTV, deltaOptimizer, errorDimensionDBF);
    MatrixDynamic jacob_expect;
    jacob_expect.resize(1, 8);
    // if all the vanishing gradient is considered:
    //  jacob_expect << 1.5, -2, -4, -1, 0.5, 0, 3.5, 1.5;
    jacob_expect << 1.5, -2, -4, -1, 0.5, 0.5, 3, 1.5;
    AssertEigenEqualMatrix(jacob_expect, jacob_actual);
}

TEST(ExtractVariable, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v20.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104;
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 0, 0));
    AssertEqualScalar(107, ExtractVariable(initialSTV, sizeOfVariables, 1, 0));
    AssertEqualScalar(5, ExtractVariable(initialSTV, sizeOfVariables, 2, 0));
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 3, 0));
    AssertEqualScalar(104, ExtractVariable(initialSTV, sizeOfVariables, 4, 0));
}
TEST(Prior_factor, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    LLint errorDimensionPrior = 1;
    vector<int> order = FindDependencyOrder(dagTasks);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);

    Prior_ConstraintFactor factor(key, dagTasks.tasks, sizeOfVariables,
                                  errorDimensionPrior, mapIndex,
                                  maskForEliminate, 0.0, order[0], model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    AssertEqualScalar(0, factor.f(startTimeVector)(0, 0));
    startTimeVector << 6, 107, 5, 3, 104, 2, 10, 101;
    AssertEqualScalar(10 * weightPrior_factor, factor.f(startTimeVector)(0, 0));
}

TEST(DAG_Optimize_schedule, v1)
{
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("../TaskData/" + testDataSetName + ".csv", "orig");

    auto sth = OptimizeScheduling(tasks);
    double success = sth.first;
    VectorDynamic res = sth.second;

    cout << "The result after optimization is " << Color::green << success << Color::blue << res << Color::def << endl;
}

// TODO: not done yet
TEST(NumericalDerivativeDynamicUpperDBF, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v22.csv", "orig");
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
    auto initialSTV = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 30.1636, 129.772, 40.7072, 0.008139, 105, 27.9779, 100, 3.072, 103.393;
    vector<bool> maskForEliminate(variableDimension, false);
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};

    Symbol key('a', 0);
    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                mapIndex, maskForEliminate, model);

    MatrixDynamic jacob_actual = factor.NumericalDerivativeDynamicUpperDBF(factor.f, initialSTV, deltaOptimizer, errorDimensionDBF);
    // cout << jacob_actual << endl;
    // MatrixDynamic jacob_expect;
    // jacob_expect.resize(1, 8);
    // // if all the vanishing gradient is considered:
    // //  jacob_expect << 1.5, -2, -4, -1, 0.5, 0, 3.5, 1.5;
    // jacob_expect << 1.5, -2, -4, -1, 0.5, 0.5, 3, 1.5;
    // AssertEigenEqualMatrix(jacob_expect, jacob_actual);
}
TEST(GenerateInitialForDAG_RelativeStart, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v17.csv", "orig");
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
    auto actual = GenerateInitialForDAG_RelativeStart(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 50, 150, 39, 27, 127, 14, 0, 100;
    assert_equal(expected, actual);
}
// TEST(graphError, v1)
// {
//     using namespace DAG_SPACE;
//     DAG_Model tasks = ReadDAG_Tasks("../TaskData/test_n5_v14.csv", "orig");
//     VectorDynamic init;
//     init.resize(5, 1);
//     init << -12.6972, 64.0853, 84.8531, 114.945, 220.221;
//     cout << "test graphErrorEvaluation: " << GraphErrorEvaluation(tasks, init) << endl;

//     cout << "test graphErrorEvaluation2: " << GraphErrorEvaluation(tasks, GenerateInitialForDAG(tasks, "orig")) << endl;
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}