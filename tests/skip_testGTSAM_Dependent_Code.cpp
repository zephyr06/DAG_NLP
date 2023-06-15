#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/testMy.h"
#include "sources/Baseline/RTSS21IC.h"
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;
TEST(DAG_Optimize_schedule, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + "test_n5_v17" + ".csv", "orig");
    auto sth = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, 1000, 1000);
    EXPECT(sth.schedulable_);
    sth = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, 10, 10);
    EXPECT(!sth.schedulable_);
}

TEST(SF, error_v1)
{

    using namespace OrderOptDAG_SPACE;
    using namespace RegularTaskSystem;
    RTSS21IC_NLP::freshTol = 0;
    RTSS21IC_NLP::sensorFusionTolerance = 0;

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial;
    initial.resize(3, 1);
    initial << 0, 2, 3;

    LLint errorDimensionSF = RTSS21IC_NLP::DAG_SPACE::CountSFError(dagTasks, tasksInfo.sizeOfVariables);
    gtsam::Symbol key('a', 0);
    RTSS21IC_NLP::MAP_Index2Data mapIndex;
    for (LLint i = 0; i < tasksInfo.variableDimension; i++)
    {
        RTSS21IC_NLP::MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    std::vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    RTSS21IC_NLP::DAG_SPACE::SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo.sizeOfVariables,
                                                                  errorDimensionSF, sensorFusionTolerance,
                                                                  mapIndex, maskForEliminate, model);

    VectorDynamic expect = GenerateVectorDynamic(errorDimensionSF);
    expect << 6, 6;
    VectorDynamic actual = factor.f(initial);
    EXPECT(assert_equal(expect, actual));
}

TEST(SF, error_v2)
{

    using namespace OrderOptDAG_SPACE;
    using namespace RegularTaskSystem;
    RTSS21IC_NLP::freshTol = 0;
    RTSS21IC_NLP::sensorFusionTolerance = 0;

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial;
    initial.resize(3, 1);
    initial << 0, 2, 3;

    LLint errorDimensionSF = RTSS21IC_NLP::DAG_SPACE::CountSFError(dagTasks, tasksInfo.sizeOfVariables);
    gtsam::Symbol key('a', 0);
    RTSS21IC_NLP::MAP_Index2Data mapIndex;
    for (LLint i = 0; i < tasksInfo.variableDimension; i++)
    {
        RTSS21IC_NLP::MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    std::vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    RTSS21IC_NLP::DAG_SPACE::SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo.sizeOfVariables,
                                                                  errorDimensionSF, sensorFusionTolerance,
                                                                  mapIndex, maskForEliminate, model);

    VectorDynamic expect = GenerateVectorDynamic(errorDimensionSF);
    expect << 1, 6, 6;
    VectorDynamic actual = factor.f(initial);
    EXPECT(assert_equal(expect, actual));
}

// The following code is from testCauseEffect.cpp

// The following test depends on GTSAM

TEST(GetStartTime, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v1.csv",
                                       "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables,
    // tasksInfo.variableDimension, initialUser);
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

    EXPECT_LONGS_EQUAL(1, GetStartTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(11, GetFinishTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(201, GetStartTime({0, 1}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(401, GetStartTime({0, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(403, GetStartTime({2, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(405, GetStartTime({4, 2}, initialEstimateFG, tasksInfo));
}

TEST(CauseAffect, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v1.csv",
                                       "orig"); // single-rate dag
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
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v2.csv",
                                       "orig"); // single-rate dag
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
    DAG_Model dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v4.csv", "orig");
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
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
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
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v13.csv", "orig");
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
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n4_v1.csv", "orig");
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

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
