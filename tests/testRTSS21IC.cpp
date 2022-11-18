#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/testMy.h"
#include "sources/Baseline/RTSS21IC.h"
using namespace gtsam;
TEST(DAG_Optimize_schedule, v1)
{
    OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + "test_n5_v17" + ".csv", "orig");
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

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
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
    vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
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

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v17.csv", "orig");
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
    vector<bool> maskForEliminate(tasksInfo.variableDimension, false);
    auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    RTSS21IC_NLP::DAG_SPACE::SensorFusion_ConstraintFactor factor(key, dagTasks, tasksInfo.sizeOfVariables,
                                                                  errorDimensionSF, sensorFusionTolerance,
                                                                  mapIndex, maskForEliminate, model);

    VectorDynamic expect = GenerateVectorDynamic(errorDimensionSF);
    expect << 1, 6, 6;
    VectorDynamic actual = factor.f(initial);
    EXPECT(assert_equal(expect, actual));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
