#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"
using namespace boost;
TEST(FindDependencyOrderDFS, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v16.csv", "orig");
    auto actual = FindDependencyOrderDFS(dagTasks);
    vector<int> expected = {4, 3, 2, 1, 0};
    AssertEqualVectorNoRepeat(expected, actual);
}
TEST(addMappingFunction, singleProcess)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
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
    MAP_Index2Data mapIndex, mapIndexExpect;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    mapIndexExpect = mapIndex;
    bool whetherEliminate;
    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);

    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 80, 160, 62.1, 50, 105, 14, 0, 100;
    auto sss = tightEliminate;
    tightEliminate = 0;
    factor.addMappingFunction(startTimeVector, whetherEliminate, forestInfo);
    mapIndexExpect[5] = {6, 14};
    mapIndexExpect[2] = {3, 12.1};
    AssertEqualMap(mapIndexExpect, forestInfo.mapIndex);
    VectorDynamic compressed = GenerateVectorDynamic(6);
    compressed << 80, 160, 50, 105, 0, 100;
    maskForEliminate[5] = true;
    maskForEliminate[2] = true;
    assert_equal(startTimeVector, RecoverStartTimeVector(compressed,
                                                         forestInfo.maskForEliminate,
                                                         forestInfo.mapIndex));
    tightEliminate = sss;
}

TEST(addMappingFunction, MultiProcess)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v29.csv", "orig");
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
    MAP_Index2Data mapIndex, mapIndexExpect;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    mapIndexExpect = mapIndex;
    bool whetherEliminate;
    TaskSetInfoDerived tasksInfo(tasks);
    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);

    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 81.1, 160, 70, 27.1, 105, 14, 0, 100;
    auto sss = tightEliminate;
    tightEliminate = 0;
    factor.addMappingFunction(startTimeVector, whetherEliminate, forestInfo);
    mapIndexExpect[0] = {2, 11.1};
    mapIndexExpect[3] = {5, 13.1};
    // mapIndexExpect[2] = {3, 12.1};
    AssertEqualMap(mapIndexExpect, forestInfo.mapIndex);
    tightEliminate = sss;
}

TEST(addMappingFunction, MultiProcessV2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v29.csv", "orig");
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
    MAP_Index2Data mapIndex, mapIndexExpect;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    mapIndexExpect = mapIndex;
    bool whetherEliminate;
    TaskSetInfoDerived tasksInfo(tasks);
    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);

    EliminationForest forestInfo(tasksInfo);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 80, 160, 62.1, 50, 105, 14, 0, 100;
    auto sss = tightEliminate;
    tightEliminate = 0;
    factor.addMappingFunction(startTimeVector, whetherEliminate, forestInfo);
    // mapIndexExpect[6] = {6, -14};
    // mapIndexExpect[2] = {3, 12.1};
    AssertEqualMap(mapIndexExpect, forestInfo.mapIndex);
    tightEliminate = sss;
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    auto sth = EstablishGraphStartTimeVector(tasksInfo);
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    auto sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    // vertex_name_map_t vertex2indexBig = get(vertex_name, g);
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension);

    vector<LLint> tree1 = {2, 3, 5, 0, 1};
    vector<LLint> tree2 = {6, 7};

    Symbol key('a', 0);
    LLint errorDimensionDBF = tasksInfo.processorTaskSet.size();

    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v18.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    auto sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    // vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    // edge_name_map_t edge2Distance = get(edge_name, g);

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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v18.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    auto sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph g = sth.first;
    indexVertexMap indexesBGL = sth.second;

    // vertex_name_map_t vertex2indexBig = get(vertex_name, g);
    // edge_name_map_t edge2Distance = get(edge_name, g);

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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
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

    vector<LLint> actual = FindVanishIndex(initialSTV, tasks, sizeOfVariables, forestInfo);
    vector<LLint> expected = {0, 5, 6};
    AssertEqualVectorNoRepeat(expected, actual);
}
// TODO: add interfaces to forestInfo
TEST(FindVanishIndex, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v17.csv", "orig");
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
    auto initialSTV = GenerateVectorDynamic(variableDimension - 1);
    initialSTV << 3, 107, 5, 3, 104, 0, 102;
    vector<bool> maskForEliminate(variableDimension, false);
    maskForEliminate[5] = true;
    MAP_Index2Data mapIndex;
    for (int i = 0; i < variableDimension; i++)
        mapIndex[i] = MappingDataStruct{i, 0};
    mapIndex[5] = MappingDataStruct{2, -3};
    forestInfo.AddLinearEliminate(5, 2, -3);

    Symbol key('a', 0);
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);

    vector<LLint> actual = FindVanishIndex(initialSTV, tasks, sizeOfVariables, forestInfo);
    vector<LLint> expected = {0, 2, 5};
    AssertEqualVectorNoRepeat(expected, actual);
}
TEST(Jacobianelimination, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v53.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks,
                                                tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initial << 174.454, 309.782, 716.536, 1132.16, 1698.74, 153.668, 399.858, 799.917, 1199.83, 1599.66, 120.021, 548.711, 1014.5, 1503.89, 4.07109, 200, 407.673, 600.044, 807.673, 1002.69, 1208.21, 1400, 1608.21, 1800, 77.5402, 503.698, 937.437, 1199.92, 1598.73;
    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    Symbol key('b', 0);
    DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF,
                                model);
    bool whetherEliminate = false;

    factor.addMappingFunction(initial, whetherEliminate, forestInfo);
    DBF_ConstraintFactor factorAfter(key, tasksInfo, forestInfo, errorDimensionDBF,
                                     model);
    VectorDynamic initialUpdate = UpdateInitialVector(initial, tasksInfo, forestInfo);

    MatrixDynamic expect = factorAfter.NumericalDerivativeDynamicUpperDBF(factorAfter.f, initialUpdate, deltaOptimizer, errorDimensionDBF);

    MatrixDynamic actual = factorAfter.DBFJacobian(factorAfter.f, initialUpdate, deltaOptimizer, errorDimensionDBF);
    assert_equal(expect, actual);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}