#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
// #include "sources/Optimization/JobOrder.h"
// #include "sources/Optimization/OptimizeOrder.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/IterationStatus.h"

using namespace OrderOptDAG_SPACE;
TEST(list_scheduling, least_finish_time_v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v74.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 1);
    std::cout << initial << std::endl;
    PrintSchedule(tasksInfo, initial);
    EXPECT(ExamDDL_Feasibility(dagTasks, tasksInfo, initial));
}

TEST(IO, ReadWriteResult)
{
    OrderOptDAG_SPACE::ScheduleResult res;
    res.schedulable_ = false;
    res.timeTaken_ = 1.1;
    res.obj_ = 2.1;
    std::string dirStr = PROJECT_PATH + "build/";
    const char *pathDataset = (dirStr).c_str();
    std::string file = "testIO.txt";
    WriteToResultFile(pathDataset, file, res, 0);
    OrderOptDAG_SPACE::ScheduleResult res2 = ReadFromResultFile(pathDataset, file, 0);
    EXPECT_DOUBLES_EQUAL(1.1, res2.timeTaken_, 1e-3);
    EXPECT_DOUBLES_EQUAL(2.1, res2.obj_, 1e-3);
    EXPECT(!res2.schedulable_);
}

TEST(ExamDBF_Feasibility, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    int processorNum = 3;
    std::vector<uint> processorJobVec;
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    PrintSchedule(tasksInfo, initial);
    EXPECT(ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    EXPECT(!ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum - 1));
    EXPECT(!ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum - 2));
}
TEST(ExamDBF_Feasibility, v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);

    std::vector<uint> processorJobVec;

    int processorNum = 2;
    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, processorJobVec);
    PrintSchedule(tasksInfo, initial);

    EXPECT(ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(0) = 1;
    EXPECT(!ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(0) = 0;
    initial(1) = 99;
    EXPECT(ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    EXPECT(!ExamDDL_Feasibility(dagTasks, tasksInfo, initial));
    initial(1) = 100;
    initial(4) = 30;
    EXPECT(ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
    initial(4) = 8;
    EXPECT(!ExamDBF_Feasibility(dagTasks, tasksInfo, initial, processorJobVec, processorNum));
}

TEST(sensorFusion, v1)
{
    using namespace OrderOptDAG_SPACE;
    using namespace RegularTaskSystem;

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial;
    initial.resize(5, 1);
    initial << 2, 1, 0, 3, 4;
    EXPECT_LONGS_EQUAL(0, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(0));

    // cout << sth << endl;
    initial << 3, 5, 1, 6, 7;
    EXPECT_LONGS_EQUAL(3, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(0));
}

TEST(sensorFusion, multi_rate)
{
    using namespace OrderOptDAG_SPACE;
    using namespace RegularTaskSystem;

    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initial;
    initial.resize(8, 1);
    initial << 6, 107,
        5,
        3, 104,
        2,
        0, 101;

    // 0 depends on 1,3,4
    // 1 finishes 5 + 11 = 16, 3 at 2+13, 4 at 101+14
    EXPECT_LONGS_EQUAL(100, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(0));
    // 1 finishes 5 + 11 = 16, 3 at 2+13, 4 at 0+14
    EXPECT_LONGS_EQUAL(2, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(1));

    // 1 depends on 2,3,4;
    // 2 finishes at 104+12, 3 at 2+13, 4 at 101+14
    EXPECT_LONGS_EQUAL(101, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(2));

    initial << 16, 107, 5, 3, 104, 2, 0, 101; // 2 finishes at 104+12, 3 at 2+13, 4 at 101+14
    // 1 finishes 5 + 11 = 16, 3 at 2+13, 4 at 0+14
    EXPECT_LONGS_EQUAL(2, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(0));
    EXPECT_LONGS_EQUAL(2, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(1));

    EXPECT_LONGS_EQUAL(101, ObtainSensorFusionError(dagTasks, tasksInfo, initial)(2));
}

// TEST(ScheduleDAGLS_LFT, v1)
// {
//     using namespace OrderOptDAG_SPACE;
//     // NumCauseEffectChain = 2;
//     int processorNum = 1;
//     considerSensorFusion = 0;
//     OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
//     scheduleOptions.processorNum_ = 1;
//     scheduleOptions.weightInMpRTDA_ = 0.5;
//     DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig", 2); // single-rate dag
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);
//     dagTasks.printChains();

//     ScheduleResult res = ScheduleDAGLS_LFT(dagTasks, processorNum, 1e4, 1e4);
//     PrintSchedule(tasksInfo, res.startTimeVector_);
//     // start Time Vector is : 0 10 21 33 46
//     // About expect: there are two chains, each chain has a max data age and a max reaction time
//     EXPECT_LONGS_EQUAL(364 * 2 + 375 * 2, res.obj_);
// }

// TEST(optimize_schedule_when_search_job_order_, v1)
// {
//     using namespace OrderOptDAG_SPACE;
//     // NumCauseEffectChain = 1;
//     int processorNum = 2;
//     considerSensorFusion = 0;
//     weightInMpRTDA = 0.5;
//     DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig", 1); // single-rate dag
//     doScheduleOptimization = 1;
//     ScheduleResult res = ScheduleDAGModel<LSchedulingFreeTA>(dagTasks, processorNum);
//     EXPECT_LONGS_EQUAL(5 + 4, res.obj_);
// }

int main()
{
    TestResult tr;
    // make sure all tests have the correct setting
    // NumCauseEffectChain = 1;
    return TestRegistry::runAllTests(tr);
}
