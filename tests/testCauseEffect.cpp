#include "unordered_map"
#include <CppUnitLite/TestHarness.h>

#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Utils/OptimizeOrderUtils.h"

using namespace OrderOptDAG_SPACE;
using namespace std;
using namespace gtsam;
using namespace GlobalVariablesDAGOpt;

TEST(RTDA, v3)
{
  using namespace OrderOptDAG_SPACE;
  OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(
      GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv", "orig");
  TaskSet tasks = dagTasks.tasks;
  TaskSetInfoDerived tasksInfo(tasks);
  int processorNum = 2;
  OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
  scheduleOptions.processorNum_ = processorNum;
  VectorDynamic initialEstimate = GenerateVectorDynamic(19);
  initialEstimate << 0, 100, 200, 321, 415, 500, 0, 100, 201, 320, 415, 500, 1, 309, 416, 202, 300, 1, 322;
  SFOrder jobOrder(tasksInfo, initialEstimate);
  std::vector<uint> processorJobVec;
  VectorDynamic expectStv =
      SimpleOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec);
  EXPECT(assert_equal(expectStv, initialEstimate));

  std::vector<int> causeEffectChain = {3, 2, 1, 0};
  auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimate);
  RTDA resM = GetMaxRTDA(res);
  EXPECT_LONGS_EQUAL(120, resM.reactionTime);
  EXPECT_LONGS_EQUAL(501, resM.dataAge); // T3_1 -> T0_2 + 600

  EXPECT(ExamAll_Feasibility(dagTasks, tasksInfo, initialEstimate, processorJobVec, processorNum, 1e9, 1e9));
}

TEST(CA, accept_reaction_within_precision)
{
  using namespace OrderOptDAG_SPACE;
  DAG_Model dagTasks =
      ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig");
  TaskSet tasks = dagTasks.tasks;
  TaskSetInfoDerived tasksInfo(tasks);
  VectorDynamic initialEstimate = GenerateVectorDynamic(4);
  initialEstimate << 0, 10, 15, 11;
  std::vector<int> causeEffectChain = {0, 2};
  auto res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimate);
  RTDA resM = GetMaxRTDA(res);
  EXPECT_LONGS_EQUAL(14, resM.reactionTime);
  EXPECT_LONGS_EQUAL(4, resM.dataAge);

  initialEstimate << 0, 10 + 1e-5, 15, 11;
  res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimate);
  resM = GetMaxRTDA(res);
  EXPECT_LONGS_EQUAL(14, resM.reactionTime);
  EXPECT_DOUBLES_EQUAL(4.0, resM.dataAge, 1e-4);
}

int main()
{
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
