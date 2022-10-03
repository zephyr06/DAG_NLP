#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Optimization/InitialEstimate.h"

TEST(GenerateInitialForDAG, V2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
    auto actual = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 6, 107, 5, 3, 104, 2, 0, 101;
    assert_equal(expected, actual);
}
TEST(GenerateInitialForDAG, V3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v22.csv", "orig");
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
    auto actual = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(9, 1);
    expected << 7, 108, 6, 4, 105, 2, 103, 0, 101;
    assert_equal(expected, actual);
}

TEST(GenerateInitialForDAG_RelativeStart, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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

TEST(RunQueue, V1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v10.csv", "orig");
    priorityMode = "RM";
    TaskSet tasks = dagTasks.tasks;

    Task tasknew = tasks[4];
    tasknew.period = 600;
    tasknew.id = 5;
    tasks.push_back(tasknew);

    RunQueue q(tasks);
    q.insert({0, 0});
    q.insert({1, 0});
    AssertEqualScalar(0, q.front());
    q.insert({5, 0});
    AssertEqualScalar(5, q.taskQueue.back().first);
    q.erase(0);
    AssertEqualScalar(1, q.front());
    q.erase(1);
    AssertEqualScalar(5, q.front());

    q.insert({4, 0});
    q.insert({2, 0});
    q.insert({3, 0});
    q.insert({1, 0});
    q.insert({0, 0});
    if (q.front() != 0 && q.front() != 4)
        CoutError("Front test failed!");
}
TEST(GenerateInitialForDAG_RM_DAG, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v14.csv", "orig");
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
    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(5, 1);
    // order from DAG dependency : 4,3,2,1,0
    expected << 21, 13, 12, 8, 0;
    assert_equal(expected, initial);
}
TEST(GenerateInitialForDAG_RM_DAG, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v23.csv", "orig");
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
    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(6, 1);
    // order from DAG dependency : 4,3,2,1,0
    expected << 21, 13, 12, 100, 8, 0;
    assert_equal(expected, initial);
}
TEST(GenerateInitialForDAG_RM_DAG, v3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
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
    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    // order from DAG dependency : 4,3,2,1,0
    // detection order at 100: 2,4,0
    expected << 50, 126, 39, 27, 100, 14, 0, 112;
    assert_equal(expected, initial);
}

TEST(GenerateInitialForDAG_RM_DAG, MultiProcessor_v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v29.csv", "orig");
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
    auto initial = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    // order from DAG dependency : 4,3,2,1,0
    // detection order at 100: 2,4,0
    expected << 50, 114, 39, 27, 100, 14, 0, 100;
    assert_equal(expected, initial);
}
TEST(GenerateInitialForDAG, Multi_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v33.csv", "orig");
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
    auto actual = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 50, 126, 39, 27, 100, 14, 0, 112;
    assert_equal(expected, actual);
}
TEST(GenerateInitialForDAG, Multi_v3_processorMap)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v34.csv", "orig");
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
    auto actual = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(10, 1);
    expected << 39,
        35,
        31, 200,
        28, 100, 200, 300,
        0, 200;
    assert_equal(expected, actual, 1e-2);
}
TEST(SimulateFixedPrioritySched, Multi_v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v34.csv", "orig");
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
    auto actual = SimulateFixedPrioritySched(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(10, 1);
    expected << 0,
        0,
        0, 200,
        0, 100, 200, 300,
        0, 200;
    assert_equal(expected, actual);
}
TEST(SimulateFixedPrioritySched, Multi_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v29.csv", "orig");
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
    auto actual = SimulateFixedPrioritySched(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 14, 114,
        24,
        0, 100,
        12,
        0, 100;
    assert_equal(expected, actual);
}
// this test is removed because float-point data file is no longer supported
// TEST(GenerateInitialForDAG_RM_DAG, v4)
// {
//     using namespace DAG_SPACE;
//     using namespace RegularTaskSystem;
//     string path = PROJECT_PATH + "TaskData/test_n3_v2.csv";
//     DAG_Model dagTasks = ReadDAG_Tasks(path, "orig");
//     TaskSet tasks = dagTasks.tasks;
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
//     VectorDynamic initialEstimate = GenerateInitialForDAG_RM_DAG(dagTasks,
//                                                                  sizeOfVariables,
//                                                                  variableDimension);
//     // cout << initialEstimate << endl;
//     VectorDynamic expect = initialEstimate;
//     expect << 2.008, 1, 2, 3, 4, 5, 6.78, 7, 8, 9, 1.765, 2, 4.01, 6.78, 8.01, 0, 5.01;
//     assert_equal(expect, initialEstimate, 0.01);
// }

TEST(SimulateFixedPrioritySched, v3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v3.csv", "orig");
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
    auto actual = SimulateFixedPrioritySched(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(17, 1);
    expected << 0, 195, 200, 300, 400, 500, 671, 700, 800, 900, 1, 201, 401, 672, 801, 25, 501;
    assert_equal(expected, actual);
}

TEST(SimulateFixedPrioritySchedDAG, v6)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v3.csv", "orig");
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
    auto actual = GenerateInitialForDAG_RM_DAG(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(17, 1);
    expected << 194, 100, 200, 300, 400, 500, 671, 700, 800, 900, 170, 201, 401, 672, 801, 0, 501;
    assert_equal(expected, actual);
}

// TEST(UpdateInitialVector, v1)
// {
//     using namespace DAG_SPACE;
//     DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     EliminationForest forestInfo(tasksInfo);

//     auto initialSTV = GenerateVectorDynamic(tasksInfo.variableDimension);
//     initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
//     forestInfo.AddLinearEliminate(5, 2, -3);
//     VectorDynamic actual = UpdateInitialVector(initialSTV, tasksInfo, forestInfo);
//     auto expect = GenerateVectorDynamic(tasksInfo.variableDimension - 1);
//     expect << 3, 107, 5, 3, 104, 0, 102;
//     AssertEigenEqualVector(expect, actual, __LINE__);
// }

TEST(toplogical, sortv1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

    TaskSet originTasks = FindSourceTasks(dagTasks);
    originTasks = Reorder(originTasks, "RM");
    std::cout << "Source tasks: ";
    for (size_t i = 0; i < originTasks.size(); i++)
    {
        std::cout << originTasks[i].id << ", ";
    }
    std::cout << std::endl;

    Graph graphBoost;
    indexVertexMap indexesBGL;
    std::tie(graphBoost, indexesBGL) = dagTasks.GenerateGraphForTaskSet();
    std::vector<int> topoOrder = TopologicalSortSingle(originTasks, dagTasks, graphBoost, indexesBGL);
    for (size_t i = 0; i < topoOrder.size(); i++)
    {
        std::cout << topoOrder[i] << std::endl;
    }
    EXPECT_LONGS_EQUAL(5, topoOrder.size());
    EXPECT_LONGS_EQUAL(0, topoOrder[0]);
    EXPECT_LONGS_EQUAL(3, topoOrder[1]);
    EXPECT_LONGS_EQUAL(4, topoOrder[2]);
}
TEST(toplogical, sortv2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

    TaskSet originTasks = FindSourceTasks(dagTasks);
    originTasks = Reorder(originTasks, "DM");
    std::cout << "Source tasks: ";
    for (size_t i = 0; i < originTasks.size(); i++)
    {
        std::cout << originTasks[i].id << ", ";
    }
    std::cout << std::endl;

    Graph graphBoost;
    indexVertexMap indexesBGL;
    std::tie(graphBoost, indexesBGL) = dagTasks.GenerateGraphForTaskSet();
    std::vector<int> topoOrder = TopologicalSortSingle(originTasks, dagTasks, graphBoost, indexesBGL);
    for (size_t i = 0; i < topoOrder.size(); i++)
    {
        std::cout << topoOrder[i] << std::endl;
    }
    EXPECT_LONGS_EQUAL(5, topoOrder.size());
    EXPECT_LONGS_EQUAL(0, topoOrder[0]);
    EXPECT_LONGS_EQUAL(4, topoOrder[1]);
    EXPECT_LONGS_EQUAL(2, topoOrder[2]);
    std::cout << "*****************************************" << std::endl;
}
TEST(toplogical, sortv3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

    auto res = TopologicalSortMulti(dagTasks);
    std::vector<int> topoOrder = res[2];
    EXPECT_LONGS_EQUAL(5, topoOrder.size());
    EXPECT_LONGS_EQUAL(0, topoOrder[0]);
    EXPECT_LONGS_EQUAL(4, topoOrder[1]);
    EXPECT_LONGS_EQUAL(2, topoOrder[2]);
}
TEST(FindSinkTaskIds, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");
    auto chains = dagTasks.FindSinkTaskIds();
    EXPECT_LONGS_EQUAL(2, chains.size());
    EXPECT_LONGS_EQUAL(0, chains[0]);
    EXPECT_LONGS_EQUAL(1, chains[1]);
}
TEST(FindSinkTaskIds, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v66.csv", "orig");
    auto chains = dagTasks.FindSinkTaskIds();
    EXPECT_LONGS_EQUAL(1, chains.size());
    EXPECT_LONGS_EQUAL(0, chains[0]);
}
TEST(get_random_chain, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");
    auto chains = dagTasks.GetRandomChains(2);
    PrintChains(chains);
    EXPECT_LONGS_EQUAL(2, chains.size());
    std::cout << std::endl;
}

TEST(GenerateInitial_Custom_DAG, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v14.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic inital = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    // for (uint i = 0; i < dagTasks.tasks.size(); i++)
    // {
    //     std::cout << dagTasks.tasks[i].priority_ << std::endl;
    // }
    EXPECT_LONGS_EQUAL(2, dagTasks.tasks[0].priority_);
    EXPECT_LONGS_EQUAL(4, dagTasks.tasks[4].priority_);
    EXPECT_LONGS_EQUAL(5, dagTasks.tasks[2].priority_);
    EXPECT_LONGS_EQUAL(3, dagTasks.tasks[3].priority_);
    EXPECT_LONGS_EQUAL(1, dagTasks.tasks[1].priority_);
    std::cout << std::endl;
}

TEST(GenerateInitial_Custom_DAG, v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic inital = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    // for (uint i = 0; i < dagTasks.tasks.size(); i++)
    // {
    //     std::cout << dagTasks.tasks[i].priority_ << std::endl;
    // }
    EXPECT_LONGS_EQUAL(5, dagTasks.tasks[0].priority_);
    EXPECT_LONGS_EQUAL(4, dagTasks.tasks[4].priority_);
    EXPECT_LONGS_EQUAL(3, dagTasks.tasks[2].priority_);
    EXPECT_LONGS_EQUAL(2, dagTasks.tasks[3].priority_);
    EXPECT_LONGS_EQUAL(1, dagTasks.tasks[1].priority_);
}

TEST(GenerateInitial_Custom_DAG, initial)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v4.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    // std::cout << initial << std::endl;
    VectorDynamic expected = initial;
    expected << 0, 100, 200, 300, 400, 500, 10, 210, 410, 21, 310;
    assert_equal(expected, initial);
    cout << std::endl;
}

TEST(GenerateInitial_Custom_DAG, n6v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    std::cout << initial << std::endl;
    VectorDynamic expected = initial;
    expected << 0, 45, 68, 5, 50, 60, 73, 85, 80, 75;
    assert_equal(expected, initial);
}

TEST(list_scheduling, least_finish_time)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    std::cout << initial << std::endl;
    PrintSchedule(tasksInfo, initial);
    VectorDynamic expected = initial;
    expected << 5, 50, 83, 10, 60, 70, 88, 0, 55, 78;
    assert_equal(expected, initial);
}

TEST(PrintSchedule, SortJobSchedule)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
    std::cout << initial << std::endl;

    std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector = ObtainAllJobSchedule(tasksInfo, initial);
    timeJobVector = SortJobSchedule(timeJobVector);
    EXPECT(JobCEC(5, 0) == timeJobVector[0].second);
    EXPECT(JobCEC(0, 0) == timeJobVector[1].second);
    EXPECT(JobCEC(1, 0) == timeJobVector[2].second);
    EXPECT(JobCEC(0, 1) == timeJobVector[3].second);
    EXPECT(JobCEC(5, 1) == timeJobVector[4].second);
    EXPECT(JobCEC(2, 0) == timeJobVector[5].second);
    EXPECT(JobCEC(3, 0) == timeJobVector[6].second);
    EXPECT(JobCEC(5, 2) == timeJobVector[7].second);
    EXPECT(JobCEC(0, 2) == timeJobVector[8].second);
    EXPECT(JobCEC(4, 0) == timeJobVector[9].second);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}