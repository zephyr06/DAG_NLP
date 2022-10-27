#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/ScheduleOptimizer.h"

using namespace OrderOptDAG_SPACE;

TEST(SFOrderSchedule, test1)
{
    using namespace OrderOptDAG_SPACE;
    // DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    for (int taskSetId = 1; taskSetId <= 79; taskSetId++)
    {
        if (taskSetId == 41)
        { // ignore task sets with double execution time
            taskSetId = 45;
        }
        DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v" + std::to_string(taskSetId) + ".csv", "orig");

        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, 2);
        SFOrder sfOrder(tasksInfo, initialSTV);
        auto startTimeVector = SFOrderScheduling(dagTasks, tasksInfo, 2, sfOrder);
        if (debugMode)
        {
            std::cout << "taskSetId is : " << taskSetId << std::endl;
            std::cout << "Initial schedule: " << std::endl;
            PrintSchedule(tasksInfo, initialSTV);
            std::cout << "SFOrder schedule: " << std::endl;
            PrintSchedule(tasksInfo, startTimeVector);
        }
        AssertEigenEqualVector(initialSTV, startTimeVector);
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
