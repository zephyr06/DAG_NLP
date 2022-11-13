#include "sources/Optimization/Optimize.h"
#include "sources/Baseline/RTSS21IC.h"
#include "sources/Tools/profilier.h"
#include "sources/Tools/testMy.h"

TEST(RTSSIC, Wang21_DBF)
{
    RTSS21IC_NLP::sensorFusionTolerance = 1e8;
    RTSS21IC_NLP::freshTol = 1e8;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v25.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    std::vector<uint> processorIdVec;
    int processorNum = 2;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, std::nullopt, processorIdVec);
    PrintSchedule(tasksInfo, initial);
    RTSS21IC_NLP::processorIdVecGlobal = processorIdVec;
    RTSS21IC_NLP::processorNumGlobal = processorNum;
    EXPECT(processorIdVec.size() > 0);
    auto initialEstimate = OrderOptDAG_SPACE::GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initial);
    double errorInitial = RTSS21IC_NLP::DAG_SPACE::GraphErrorEvaluation(dagTasks, initialEstimate);
    EXPECT(ExamAll_Feasibility(dagTasks, tasksInfo, initial,
                               processorIdVec, processorNum, RTSS21IC_NLP::sensorFusionTolerance,
                               RTSS21IC_NLP::freshTol));
    EXPECT_LONGS_EQUAL(0, errorInitial);
}

// TEST(DAG_Optimize_schedule, v1)
// {
//     BeginTimer("main");
//     using namespace OrderOptDAG_SPACE;
//     DAG_Model tasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

//     auto sth = ScheduleRTSS21IC(tasks, sensorFusionTolerance, freshTol);
//     EndTimer("main");
//     PrintTimer();
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}