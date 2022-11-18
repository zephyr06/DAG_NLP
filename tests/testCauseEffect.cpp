#include "unordered_map"
#include <CppUnitLite/TestHarness.h>

#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Utils/OptimizeOrderUtils.h"

using namespace OrderOptDAG_SPACE;
using namespace std;
// job data structure used in cause-effect chain analysis

// void AddReactionTimeDataAgeFactor(NonlinearFactorGraph &graph,
//                                   TaskSetInfoDerived &tasksInfo,
//                                   const std::vector<int> &causeEffectChain)
// {

//     if (RtdaWeight == 0)
//         return;

//     std::vector<gtsam::Symbol> keysAll;
//     keysAll.reserve(tasksInfo.length);
//     for (uint i = 0; i < tasksInfo.tasks.size(); i++)
//     {
//         for (size_t j = 0; j < static_cast<size_t>(tasksInfo.hyperPeriod / tasksInfo.tasks[i].period); j++)
//         {
//             keysAll.push_back(GenerateKey(i, j));
//         }
//     }

//     LambdaMultiKey f = [keysAll, tasksInfo, causeEffectChain](const Values &x)
//     {
//         LLint hyperPeriod = tasksInfo.hyperPeriod;
//         const TaskSet &tasks = tasksInfo.tasks;
//         size_t totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period;

//         VectorDynamic res = GenerateVectorDynamic(totalStartJobs * 2);

//         std::unordered_map<JobCEC, JobCEC> firstReactionMap;
//         // Todo: Be careful! this termination condition is not consistent with Verucchi!
//         for (size_t startInstanceIndex = 0; startInstanceIndex <= totalStartJobs + 1; startInstanceIndex++)
//         {

//             JobCEC firstJob = {causeEffectChain[0], 0};
//             JobCEC lastJob = {-1, 0};
//             bool findLastJob = false;
//             for (uint j = 1; j < causeEffectChain.size(); j++)
//             {
//                 double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
//                 size_t jobIndex = 0;
//                 while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
//                 {
//                     jobIndex++;
//                     if (jobIndex > 100)
//                     {
//                         CoutError("didn't find a match!");
//                     }
//                 }
//                 firstJob = {causeEffectChain[j], jobIndex};
//                 if (jobIndex > 0)
//                 {
//                     lastJob = {causeEffectChain[j], jobIndex - 1};
//                     findLastJob = true;
//                 }
//             }

//             // TODO: Be careful, whether using < or <=
//             if (startInstanceIndex < totalStartJobs)
//             {
//                 JobCEC jj(causeEffectChain[0], size_t(0));
//                 firstReactionMap[jj] = firstJob;
//                 res(startInstanceIndex * 2, 0) = GetFinishTime(firstJob, x, tasksInfo) - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);
//             }

//             if (findLastJob && startInstanceIndex > 0)
//             {
//                 auto p = firstReactionMap.find({causeEffectChain[0], startInstanceIndex - 1});
//                 if (p != firstReactionMap.end() && (p->second) != firstJob)
//                 {
//                     res(startInstanceIndex * 2 - 1, 0) = GetStartTime(lastJob, x, tasksInfo) + tasks[lastJob.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
//                 }
//             }
//         }

//         return res;
//     };

//     LLint errorDimensionRTDA = 2;
//     auto model = noiseModel::Isotropic::Sigma(errorDimensionRTDA, noiseModelSigma / RtdaWeight);

//     graph.emplace_shared<MultiKeyFactor>(keysAll, f, tasksInfo.hyperPeriod / tasksInfo.tasks[causeEffectChain[0]].period * 2, model);
// }

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
    VectorDynamic initialEstimate = GenerateVectorDynamic(19);
    initialEstimate << 0, 100, 200, 321, 415, 500, 0, 100, 201, 320, 415, 500, 1, 309, 416, 202, 300, 1, 322;
    SFOrder jobOrder(tasksInfo, initialEstimate);
    std::vector<uint> processorJobVec;
    VectorDynamic expectStv = SimpleOrderScheduler::schedule(dagTasks, tasksInfo, processorNum, jobOrder, processorJobVec);
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
