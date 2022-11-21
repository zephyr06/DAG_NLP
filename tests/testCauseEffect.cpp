#include "unordered_map"
#include <CppUnitLite/TestHarness.h>

#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Utils/OptimizeOrderUtils.h"

using namespace OrderOptDAG_SPACE;
using namespace std;
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;
TEST(GetStartTime, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

    EXPECT_LONGS_EQUAL(1, GetStartTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(11, GetFinishTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(201, GetStartTime({0, 1}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(401, GetStartTime({0, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(403, GetStartTime({2, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(405, GetStartTime({4, 2}, initialEstimateFG, tasksInfo));
    // EXPECT_LONGS_EQUAL(405, GetStartTime({5, 2}, initialEstimateFG, tasksInfo));
}

TEST(CauseAffect, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    EXPECT_LONGS_EQUAL(414, resM.reactionTime);
    EXPECT_LONGS_EQUAL(414, resM.dataAge);
}
TEST(CA, V2)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v2.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initialEstimate = GenerateVectorDynamic(6);
    initialEstimate << 1, 101, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    EXPECT_LONGS_EQUAL(414, resM.reactionTime);
    EXPECT_LONGS_EQUAL(314, resM.dataAge);
}

TEST(CA, v3)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v4.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GenerateVectorDynamic(11);
    initialEstimate << 1, 101, 202, 303, 404, 505, 2, 204, 406, 3, 306;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    EXPECT_LONGS_EQUAL(514, resM.reactionTime);
    EXPECT_LONGS_EQUAL(312, resM.dataAge);
}

TEST(CA_customize, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GenerateVectorDynamic(10);
    initialEstimate << 5, 57, 85, 10, 62, 72, 50, 0, 52, 80;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 4, 5};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    resM.print();
    // EXPECT_LONGS_EQUAL(514, resM.reactionTime);
    // EXPECT_LONGS_EQUAL(312, resM.dataAge);
}

TEST(RTDA, v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v13.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GenerateVectorDynamic(7);
    initialEstimate << 0, 37, 100, 200, 12, 100, 200;
    PrintSchedule(tasksInfo, initialEstimate);
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {2, 1};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    std::cout << std::endl;
    // for (auto rtda : res)
    // {
    //     rtda.print();
    // }
    EXPECT_LONGS_EQUAL(4, res.size());
    EXPECT_LONGS_EQUAL(37, res[0].reactionTime);
    EXPECT_LONGS_EQUAL(149, res[2].reactionTime);
    EXPECT_LONGS_EQUAL(-1, res[2].dataAge);
    EXPECT_LONGS_EQUAL(100, res[3].dataAge);
    RTDA resM = GetMaxRTDA(res);
    std::cout << std::endl;
    resM.print();
}
TEST(RTDA, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n4_v1.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GenerateVectorDynamic(7);
    initialEstimate << 3, 4, 2, 5, 1, 6, 7;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    EXPECT_LONGS_EQUAL(4, resM.reactionTime);
    EXPECT_LONGS_EQUAL(6, resM.dataAge);
}

TEST(RTDA, v3)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v79.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    int processorNum = 2;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
    scheduleOptions.processorNum_ = processorNum;
    VectorDynamic initialEstimate = GenerateVectorDynamic(19);
    initialEstimate << 0, 100, 200, 321, 415, 500, 0, 100, 201, 320, 415, 500, 1, 309, 416, 202, 300, 1, 322;
    SFOrder jobOrder(tasksInfo, initialEstimate);
    std::vector<uint> processorJobVec;
    VectorDynamic expectStv = SimpleOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
    EXPECT(assert_equal(expectStv, initialEstimate));

    std::vector<int> causeEffectChain = {3, 2, 1, 0};
    auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimate);
    RTDA resM = GetMaxRTDA(res);
    EXPECT_LONGS_EQUAL(120, resM.reactionTime);
    EXPECT_LONGS_EQUAL(501, resM.dataAge); // T3_1 -> T0_2 + 600

    EXPECT(ExamAll_Feasibility(dagTasks, tasksInfo, initialEstimate, processorJobVec, processorNum, 1e9, 1e9));
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
