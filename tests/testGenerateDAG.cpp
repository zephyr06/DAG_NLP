

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sources/Optimization/OptimizeSFOrder.h"
using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class DAGScheduleOptimizerTest1 : public ::testing::Test {
protected:
  void SetUp() override {
    dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv",
                             "orig"); // single-rate dag
    tasks = dagTasks.tasks;
    tasksInfo = TaskSetInfoDerived(tasks);
    chain1 = {0, 2};
    dagTasks.chains_[0] = chain1;
    timeLimits = 1;
    scheduleOptions.processorNum_ = 2;
    scheduleOptions.considerSensorFusion_ = 0;
    scheduleOptions.freshTol_ = 0;
    scheduleOptions.sensorFusionTolerance_ = 0;
    scheduleOptions.weightInMpRTDA_ = 0.5;
    scheduleOptions.weightInMpSf_ = 0.5;
    scheduleOptions.weightPunish_ = 10;

    dagScheduleOptimizer =
        DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj>(dagTasks, scheduleOptions, timeLimits);
  };

  double timeLimits = 1;
  DAG_Model dagTasks;
  TaskSet tasks;
  TaskSetInfoDerived tasksInfo;
  std::vector<int> chain1;
  ScheduleOptions scheduleOptions;
  DAGScheduleOptimizer<SimpleOrderScheduler, RTDAExperimentObj> dagScheduleOptimizer;
};

struct ChainParameter {
  std::vector<int> active_patterns;
  std::vector<int> runnable_per_pattern;

  inline int GetTaskNum() const {
    return std::accumulate(runnable_per_pattern.begin(), runnable_per_pattern.end(), 0);
  }
};

namespace WATERS15 {
// Table VI. in WATERS15
size_t GetNumberOfActivationPattern() {
  double random_num = (double)rand() / (RAND_MAX + 1);
  if (random_num < 0.7)
    return 1;
  else if (random_num < 0.9)
    return 2;
  else
    return 3;
}

// Table VII. in WATERS15
size_t GetRandomNumberOfTasksForSinglePeriod() {
  double random_num = (double)rand() / (RAND_MAX + 1);
  if (random_num < 0.3)
    return 2;
  else if (random_num < 0.7)
    return 3;
  else if (random_num < 0.2)
    return 4;
  else
    return 5;
}

std::vector<int> GetUniquePeriod(const TaskSet &tasks) {
  std::vector<int> periods;
  periods.reserve(tasks.size());
  for (uint i = 0; i < tasks.size(); i++)
    periods.push_back(tasks[i].period);
  return periods;
}

std::vector<int> selectRandomElements(const std::vector<int> &inputVector, int numElements) {
  std::vector<int> result;

  if (numElements <= 0 || numElements > inputVector.size()) {
    std::cerr << "Invalid number of elements requested.\n";
    return result;
  }

  // std::srand(std::time(nullptr)); // Seed the random number generator with the current time

  std::vector<int> tempVector = inputVector; // Make a copy of the input vector to work with

  for (int i = 0; i < numElements; ++i) {
    int randomIndex = std::rand() % tempVector.size(); // Generate a random index
    result.push_back(tempVector[randomIndex]);         // Add the randomly selected element to the result
    tempVector.erase(tempVector.begin() +
                     randomIndex); // Remove the selected element from the temporary vector
  }

  return result;
}

inline std::vector<int> GetRandomUniquePeriod(const TaskSet &tasks, size_t activation_pattern_num) {
  std::vector<int> unique_period = GetUniquePeriod(tasks);
  return selectRandomElements(unique_period, activation_pattern_num);
}

std::vector<int> SelectAllTasksOfGivenPeriod(const DAG_Model &dagModel, int period) {
  std::vector<int> res;
  for (uint i = 0; i < dagModel.tasks.size(); i++) {
    if (dagModel.tasks[i].period == period)
  }
  res.push_back(dagModel.tasks[i].taskId);
  return res;
}

std::vector<int> SelectTasksOfGivenPeriod(const DAG_Model &dagModel, int period, int task_num) {
  std::vector<int> all_tasks_given_period = SelectAllTasksOfGivenPeriod(dagModel, period);
  if (all_tasks_given_period.size() < task_num)
    return {};
}
std::vector<int> SelectRandomElements(const std::vector<int> &tasks_curr, int task_num) {
  std::vector<int> tasks_selected = tasks_curr;
  std::shuffle(tasks_selected.begin(), tasks_selected.end());
  tasks_selected.resize(task_num);
  return tasks_selected;
}
TaskSet GenerateRandomCauseEffectChain(const DAG_Model &dagModel,
                                       const std::vector<int> &activation_patterns) {
  // go through each period in activation_patterns, generate a random number 2~5, and select these tasks;

  std::unordered_set<int> tasks_included_in_chain;
  for (int period_curr : activation_patterns) {
    int task_num = GetRandomNumberOfTasksForSinglePeriod();
    std::vector<int> tasks_curr = SelectTasksOfGivenPeriod(dagModel, period_curr, task_num);
    std::vecto<int> tasks_selected = SelectRandomElements(tasks_curr, task_num);
    if (tasks_selected.size() == 0)
      return {};
    tasks_included_in_chain.insert(tasks_selected.begin(), tasks_selected.end());
  }
  return std::vector<int>(tasks_included_in_chain.begin(), tasks_included_in_chain.end());
}

} // namespace WATERS15

DAG_Model GenerateDAG_WATERS15(int N, double totalUtilization, int numberOfProcessor, int periodMin,
                               int periodMax, int coreRequireMax, int taskSetType, int deadlineType,
                               size_t total_try_allowed = 1e3) {
  using namespace WATERS15;
  size_t chain_number = N * 3;
  TaskSet tasks = GenerateTaskSet(N, totalUtilization, numberOfProcessor, periodMin, periodMax,
                                  coreRequireMax, taskSetType, deadlineType);

  MAP_Prev mapPrev; // initialize the data structure to save edges
  DAG_Model dagModel(tasks, mapPrev);
  //    dagModel.addEdge(i, j);

  size_t try_count = 0;
  std::vector<ChainParameter> chain_param = GetRandomChains();
  for (uint chain_index = 0; chain_index < chain_number && try_count < total_try_allowed; chain_index++) {
    std::vector<int> activation_patterns = GetRandomUniquePeriod(tasks, GetNumberOfActivationPattern());
    // the chains are not ordered
    TaskSet tasks_for_a_chain = GenerateRandomCauseEffectChain(tasks, activation_patterns);

    AddCauseEffectChain(dagModel, chains);
    try_count++;
  }

  return dagModel;
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}