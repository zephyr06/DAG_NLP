#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Utils/testMy.h"
#include "sources/Optimization/InitialEstimate.h"
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;

TEST(RunQueue, V1)
{
    using namespace OrderOptDAG_SPACE;
    // priorityMode = "RM";
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v10.csv", "RM");
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
TEST(toplogical, sortv1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

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
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

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
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");

    auto res = TopologicalSortMulti(dagTasks);
    std::vector<int> topoOrder = res[2];
    EXPECT_LONGS_EQUAL(5, topoOrder.size());
    EXPECT_LONGS_EQUAL(0, topoOrder[0]);
    EXPECT_LONGS_EQUAL(4, topoOrder[1]);
    EXPECT_LONGS_EQUAL(2, topoOrder[2]);
}
TEST(FindSinkTaskIds, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");
    auto chains = dagTasks.FindSinkTaskIds();
    EXPECT_LONGS_EQUAL(2, chains.size());
    EXPECT_LONGS_EQUAL(0, chains[0]);
    EXPECT_LONGS_EQUAL(1, chains[1]);
}
TEST(FindSinkTaskIds, v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v66.csv", "orig");
    auto chains = dagTasks.FindSinkTaskIds();
    EXPECT_LONGS_EQUAL(1, chains.size());
    EXPECT_LONGS_EQUAL(0, chains[0]);
}
TEST(get_random_chain, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v40.csv", "orig");
    std::vector<std::vector<int>> chains = dagTasks.GetRandomChains(2);
    OrderOptDAG_SPACE::PrintChains(chains);
    EXPECT_LONGS_EQUAL(2, chains.size());
    std::cout << std::endl;
}

// TEST(GenerateInitial_Custom_DAG, v1)
// {
//     using namespace OrderOptDAG_SPACE;
//     OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +  "TaskData/test_n5_v14.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     VectorDynamic inital = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
//     // for (uint i = 0; i < dagTasks.tasks.size(); i++)
//     // {
//     //     std::cout << dagTasks.tasks[i].priority_ << std::endl;
//     // }
//     EXPECT_LONGS_EQUAL(2, dagTasks.tasks[0].priority_);
//     EXPECT_LONGS_EQUAL(4, dagTasks.tasks[4].priority_);
//     EXPECT_LONGS_EQUAL(5, dagTasks.tasks[2].priority_);
//     EXPECT_LONGS_EQUAL(3, dagTasks.tasks[3].priority_);
//     EXPECT_LONGS_EQUAL(1, dagTasks.tasks[1].priority_);
//     std::cout << std::endl;
// }

// TEST(GenerateInitial_Custom_DAG, v2)
// {
//     using namespace OrderOptDAG_SPACE;
//     OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +  "TaskData/test_n5_v40.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     VectorDynamic inital = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
//     // for (uint i = 0; i < dagTasks.tasks.size(); i++)
//     // {
//     //     std::cout << dagTasks.tasks[i].priority_ << std::endl;
//     // }
//     EXPECT_LONGS_EQUAL(5, dagTasks.tasks[0].priority_);
//     EXPECT_LONGS_EQUAL(4, dagTasks.tasks[4].priority_);
//     EXPECT_LONGS_EQUAL(3, dagTasks.tasks[2].priority_);
//     EXPECT_LONGS_EQUAL(2, dagTasks.tasks[3].priority_);
//     EXPECT_LONGS_EQUAL(1, dagTasks.tasks[1].priority_);
// }

// TEST(GenerateInitial_Custom_DAG, initial)
// {
//     using namespace OrderOptDAG_SPACE;
//     OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +  "TaskData/test_n3_v4.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     VectorDynamic initial = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
//     // std::cout << initial << std::endl;
//     VectorDynamic expected = initial;
//     expected << 0, 100, 200, 300, 400, 500, 10, 210, 410, 21, 310;
//     assert_equal(expected, initial);
//    std::cout << std::endl;
// }

// TEST(GenerateInitial_Custom_DAG, n6v1)
// {
//     using namespace OrderOptDAG_SPACE;
//     OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH +  "TaskData/test_n6_v1.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     VectorDynamic initial = GenerateInitial_Custom_DAG(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, 0);
//     std::cout << initial << std::endl;
//     VectorDynamic expected = initial;
//     expected << 0, 45, 68, 5, 50, 60, 73, 85, 80, 75;
//     assert_equal(expected, initial);
// }

TEST(list_scheduling, least_finish_time)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n6_v1.csv", "RM");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    std::cout << initial << std::endl;
    PrintSchedule(tasksInfo, initial);
    VectorDynamic expected = initial;
    expected << 5, 55, 83, 10, 60, 70, 88, 0, 50, 78;
    EXPECT(gtsam::assert_equal(expected, initial));
}

TEST(PrintSchedule, SortJobSchedule)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n6_v1.csv", "RM");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    std::cout << initial << std::endl;

    std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector = ObtainAllJobSchedule(tasksInfo, initial);
    timeJobVector = SortJobSchedule(timeJobVector);
    EXPECT(JobCEC(5, 0) == timeJobVector[0].second);
    EXPECT(JobCEC(0, 0) == timeJobVector[1].second);
    EXPECT(JobCEC(1, 0) == timeJobVector[2].second);
    EXPECT(JobCEC(5, 1) == timeJobVector[3].second);
    EXPECT(JobCEC(0, 1) == timeJobVector[4].second);
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