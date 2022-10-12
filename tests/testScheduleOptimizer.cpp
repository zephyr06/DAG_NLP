#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/OptimizeOrder.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Tools/profilier.h"

using namespace DAG_SPACE;

TEST(JobCEC, GetJobUniqueId)
{
    DAG_Model dag_tasks();
    TaskSet tasks;
    tasks.push_back(RegularTaskSystem::Task{0, 100, 0, 1, 100});
    tasks.push_back(RegularTaskSystem::Task{0, 200, 0, 2, 200});
    tasks.push_back(RegularTaskSystem::Task{0, 300, 0, 3, 300});

    TaskSetInfoDerived tasksInfo(tasks);

    EXPECT(JobCEC(0, 0) == GetJobCECFromUniqueId(0, tasksInfo));
    EXPECT(JobCEC(0, 2) == GetJobCECFromUniqueId(2, tasksInfo));
    EXPECT(JobCEC(0, 4) == GetJobCECFromUniqueId(4, tasksInfo));
    EXPECT(JobCEC(1, 0) == GetJobCECFromUniqueId(6, tasksInfo));
    EXPECT(JobCEC(1, 2) == GetJobCECFromUniqueId(8, tasksInfo));
    EXPECT(JobCEC(2, 1) == GetJobCECFromUniqueId(10, tasksInfo));

    EXPECT(1 == GetJobUniqueId(JobCEC(0, 1), tasksInfo));
    EXPECT(3 == GetJobUniqueId(JobCEC(0, 3), tasksInfo));
    EXPECT(5 == GetJobUniqueId(JobCEC(0, 5), tasksInfo));
    EXPECT(7 == GetJobUniqueId(JobCEC(1, 1), tasksInfo));
    EXPECT(9 == GetJobUniqueId(JobCEC(2, 0), tasksInfo));

    EXPECT(5 == GetJobUniqueId(JobCEC(0, 11), tasksInfo));
    EXPECT(7 == GetJobUniqueId(JobCEC(1, 4), tasksInfo));
    EXPECT(9 == GetJobUniqueId(JobCEC(2, 2), tasksInfo));
}

TEST(ScheduleOptimizer, single_core_optimization)
{
    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v10.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 0, 11, 20, 1, 6;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    auto res = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    resM.print();
    EXPECT_LONGS_EQUAL(30, resM.reactionTime);
    EXPECT_LONGS_EQUAL(11, resM.dataAge);
    ScheduleResult result_to_be_optimized;
    ScheduleResult result_after_optimization;

    result_to_be_optimized.startTimeVector_ = initialEstimate;
    result_to_be_optimized.rtda_ = resM;
    schedule_optimizer.Optimize(dagTasks, result_to_be_optimized);
    result_after_optimization = schedule_optimizer.getOptimizedResult();
    result_after_optimization.rtda_.print();
    EXPECT_LONGS_EQUAL(22, result_after_optimization.rtda_.reactionTime);
    EXPECT_LONGS_EQUAL(11, result_after_optimization.rtda_.dataAge);
}

TEST(ScheduleOptimizer, multi_core_optimization)
{
    std::cout << "\n\n#############  New Test  ##############\n\n";
    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n3_v8.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
    PrintSchedule(tasksInfo, initial);
    JobOrderMultiCore jobOrder(tasksInfo, initial);

    std::vector<uint> processorJobVec;

    initial = ListSchedulingLFTPA(dagTasks, tasksInfo, 2, jobOrder, processorJobVec);
    VectorDynamic actualAssignment = Vector2Eigen<uint>(processorJobVec);
    VectorDynamic expected = actualAssignment;
    expected << 0, 0, 1, 1, 0;
    EXPECT(assert_equal(expected, actualAssignment));
    Values initialEstimateFG = GenerateInitialFG(initial, tasksInfo);
    auto res = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], initialEstimateFG);
    RTDA resM = GetMaxRTDA(res);
    resM.print();
    EXPECT_LONGS_EQUAL(340, resM.reactionTime);
    EXPECT_LONGS_EQUAL(240, resM.dataAge);

    ScheduleResult result_to_be_optimized;
    ScheduleResult result_after_optimization;

    result_to_be_optimized.startTimeVector_ = initial;
    result_to_be_optimized.rtda_ = resM;
    result_to_be_optimized.processorJobVec_ = processorJobVec;
    schedule_optimizer.Optimize(dagTasks, result_to_be_optimized);
    result_after_optimization = schedule_optimizer.getOptimizedResult();
    PrintSchedule(tasksInfo, result_after_optimization.startTimeVector_);
    result_after_optimization.rtda_.print();
    EXPECT_LONGS_EQUAL(250, result_after_optimization.rtda_.reactionTime);
    EXPECT_LONGS_EQUAL(240, result_after_optimization.rtda_.dataAge);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
