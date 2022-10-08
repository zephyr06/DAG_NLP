#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
// #include "sources/Optimization/ScheduleOptimizer.h"

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
}

// TEST(ScheduleOptimizer, constructor)
// {
//     ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
//     schedule_optimizer.print();
//     schedule_optimizer.SolveLP();
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
