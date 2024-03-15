
#include "sources/Baseline/SimulatedAnnealing/OptimizeSA.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace std;
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;

TEST(GetJobMinMaxStartTimeRange, V1) {
  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v3.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();

  RegularTaskSystem::TaskSetInfoDerived tasks_info(dagTasks.tasks);
  std::vector<uint> processorJobVec;
  VectorDynamic initialEstimate =
      ListSchedulingLFTPA(dagTasks, tasks_info, scheduleOptions.processorNum_, processorJobVec);
  std::cout << initialEstimate << std::endl;
  auto res = GetJobMinMaxStartTimeRange(tasks_info);
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(100 * i, res[i].first);
    EXPECT_EQ(100 * (i + 1) - 10, res[i].second);
  }
  for (int i = 4; i < 6; i++) {
    EXPECT_EQ(200 * (i - 4), res[i].first);
    EXPECT_EQ(200 * (i - 4 + 1) - 11, res[i].second);
  }
}

TEST(run_sa, v1) {

  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v3.csv", "orig");
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