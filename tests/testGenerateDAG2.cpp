

#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/GenerateRandomTaskset.h"
#include "sources/TaskModel/GenerateRandomTasksetWATERS.h"

#include <algorithm>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <vector>
using namespace OrderOptDAG_SPACE;
// using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

class DAGScheduleOptimizerTest1 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n10_v5.csv", "orig");
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
  };

  double timeLimits = 1;
  DAG_Model dagTasks;
  TaskSet tasks;
  TaskSetInfoDerived tasksInfo;
};

TEST_F(DAGScheduleOptimizerTest1, RandomSelectChains) {
  std::vector<std::vector<int>> chains = {{0, 2}, {1, 2}};
  RandomSelector selector(dagTasks, chains);
  selector.selection_chance = {1, 0};
  auto res = selector.RandomSelectChains(1);
  EXPECT_EQ(1, res.size());

  selector.selection_chance = {0, 0};
  res = selector.RandomSelectChains(1);
  EXPECT_EQ(0, res.size());
}
TEST_F(DAGScheduleOptimizerTest1, AssignSelectionChance_single_period) {
  std::vector<std::vector<int>> chains = {{0, 1}, {0, 1, 2}, {0, 1, 2, 7}, {0, 1, 2, 7, 3}};
  RandomSelector selector(dagTasks, chains);
  EXPECT_NEAR(0.3, selector.selection_chance[0], 0.01);
  EXPECT_NEAR(0.4, selector.selection_chance[1], 0.01);
  EXPECT_NEAR(0.2, selector.selection_chance[2], 0.01);
  EXPECT_NEAR(0.1, selector.selection_chance[3], 0.01);
}
TEST_F(DAGScheduleOptimizerTest1, AssignSelectionChance_two_period) {
  std::vector<std::vector<int>> chains = {{0, 4}, {0, 1, 4}, {0, 1, 2, 4, 6}, {0, 1, 4, 6}};
  RandomSelector selector(dagTasks, chains);
  EXPECT_EQ(0, selector.selection_chance[0]);
  EXPECT_EQ(0, selector.selection_chance[1]);
  EXPECT_NEAR(0.4 * 0.3 / (0.3 * 0.3 + 0.3 * 0.4), selector.selection_chance[2], 0.01);
  EXPECT_NEAR(0.3 * 0.3 / (0.3 * 0.3 + 0.3 * 0.4), selector.selection_chance[3], 0.01);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}