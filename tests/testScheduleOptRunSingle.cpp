#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/OptimizeOrder.h"
#include "sources/Tools/profilier.h"

using namespace OrderOptDAG_SPACE;

TEST(ScheduleOptimizer, RunSingle)
{
    BeginTimer("main");
    using namespace OrderOptDAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    ScheduleResult sth;
    if (processorAssignmentMode == 0)
        sth = OrderOptDAG_SPACE::ScheduleDAGModel<LSchedulingKnownTA>(dagTasks);
    else if (processorAssignmentMode == 1)
        sth = OrderOptDAG_SPACE::ScheduleDAGModel<LSchedulingFreeTA>(dagTasks);
    PrintResultAnalyzation(sth, dagTasks);
    std::cout << "Schedulable? " << sth.schedulable_ << std::endl;
    EndTimer("main");

    BeginTimer("schedule_optimizer");
    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
    schedule_optimizer.Optimize(dagTasks, sth);
    auto result_after_optimization = schedule_optimizer.getOptimizedResult();
    EndTimer("schedule_optimizer");
    dagTasks.printChains();
    result_after_optimization.print();
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
