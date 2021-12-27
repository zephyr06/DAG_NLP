#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

TEST(GenerateInitialForDAG, V2)
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
    auto actual = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 6, 107, 5, 3, 104, 2, 0, 101;
    assert_equal(expected, actual);
}
TEST(GenerateInitialForDAG, V3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v22.csv", "orig");
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
    auto actual = GenerateInitialForDAG_RelativeStart(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(8, 1);
    expected << 50, 150, 39, 27, 127, 14, 0, 100;
    assert_equal(expected, actual);
}

TEST(RunQueue, V1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v10.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v14.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v23.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v29.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v33.csv", "orig");
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
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v34.csv", "orig");
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
TEST(GenerateInitial_RM, Multi_v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v34.csv", "orig");
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
    auto actual = GenerateInitial_RM(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(10, 1);
    expected << 0,
        0,
        0, 200,
        0, 100, 200, 300,
        0, 200;
    assert_equal(expected, actual);
}
TEST(GenerateInitial_RM, Multi_v2)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v29.csv", "orig");
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
    auto actual = GenerateInitial_RM(dagTasks, sizeOfVariables, variableDimension);
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
//     string path = "/home/zephyr/Programming/DAG_NLP/TaskData/test_n3_v2.csv";
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

TEST(GenerateInitial_RM, v3)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n3_v3.csv", "orig");
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
    auto actual = GenerateInitial_RM(dagTasks, sizeOfVariables, variableDimension);
    VectorDynamic expected;
    expected.resize(17, 1);
    expected << 0, 195, 200, 300, 400, 500, 671, 700, 800, 900, 1, 201, 401, 672, 801, 25, 501;
    assert_equal(expected, actual);
}

TEST(GenerateInitial_RMDAG, v6)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n3_v3.csv", "orig");
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

TEST(UpdateInitialVector, v1)
{
    using namespace DAG_SPACE;
    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto initialSTV = GenerateVectorDynamic(tasksInfo.variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    forestInfo.AddLinearEliminate(5, 2, -3);
    VectorDynamic actual = UpdateInitialVector(initialSTV, tasksInfo, forestInfo);
    auto expect = GenerateVectorDynamic(tasksInfo.variableDimension - 1);
    expect << 3, 107, 5, 3, 104, 0, 102;
    AssertEigenEqualVector(expect, actual, __LINE__);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}