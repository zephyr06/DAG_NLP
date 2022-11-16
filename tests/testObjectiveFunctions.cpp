#include <gtest/gtest.h>
#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Optimization/ObjectiveFunctions.h"
using namespace OrderOptDAG_SPACE;

class RTDAExperimentObjTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig"); // single-rate dag
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
    }
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
};

TEST_F(RTDAExperimentObjTest1, constructor)
{
    EXPECT_EQ(tasks.size(), 5);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}