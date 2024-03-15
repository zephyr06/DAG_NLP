
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

TEST(GenerateRandomStartTimes, V1) {
  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v3.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  RegularTaskSystem::TaskSetInfoDerived tasks_info(dagTasks.tasks);

  moe::SimulatedAnnealing<double> moether(moe::SAParameters<double>()
                                              .withTemperature(GlobalVariablesDAGOpt::temperatureSA)
                                              .withCoolingRate(GlobalVariablesDAGOpt::coolingRateSA)
                                              .withDimensions(tasks_info.variableDimension + 1)
                                              .withRange({0, double(tasks_info.hyper_period)}));
  moether.AddJobMinMaxStartTimeRange(GetJobMinMaxStartTimeRange(tasks_info));
  vector<double> one_random_start_time = moether.GenerateRandomStartTimes();
  for (int x : one_random_start_time)
    cout << x << "\n";
  for (int i = 0; i < tasks_info.variableDimension; i++) {
    EXPECT_GE(one_random_start_time[i], moether.job_min_max_start_time_range_[i].first);
    EXPECT_LE(one_random_start_time[i], moether.job_min_max_start_time_range_[i].second);
  }
}

TEST(run_sa, v1) {

  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  auto res = OptimizeSchedulingSA<ReactionTimeObj>(dagTasks, scheduleOptions, 1);
  cout << res.obj_ << "\n";
  EXPECT_TRUE(res.schedulable_);
  EXPECT_LE(res.obj_, 23); // 23 is initial solution in case of 4 cores
}

TEST(run_sa, v2_RT) {
  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n10_v5.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  auto res = OptimizeSchedulingSA<ReactionTimeObj>(dagTasks, scheduleOptions, 3);
  std::cout<<"Reaction time: " << res.obj_ << std::endl;
  EXPECT_TRUE(res.schedulable_);
}
TEST(run_sa, v2_DA) {
  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n10_v5.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  auto res = OptimizeSchedulingSA<DataAgeObj>(dagTasks, scheduleOptions, 3);
  std::cout<<"Data age: " << res.obj_ << std::endl;
  EXPECT_TRUE(res.schedulable_);
}
TEST(run_sa, v2_SF) {
  auto dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n10_v5.csv", "orig");
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.LoadParametersYaml();
  auto res = OptimizeSchedulingSA<SensorFusionObj>(dagTasks, scheduleOptions, 3);
  std::cout<<"Time disparity: " << res.obj_ << std::endl;
  EXPECT_TRUE(res.schedulable_);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}