
#include "sources/Baseline/SimulatedAnnealing/OptimizeSA.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace std;
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

TEST(run_sa, v1) {

  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n10_v5.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  auto res = OptimizeSchedulingSA<ReactionTimeObj>(dagTasks, scheduleOptions, 5);
  cout << res.obj_ << "\n";
  EXPECT_TRUE(res.schedulable_);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}