

#include "sources/Optimization/OptimizeSFOrder.h"
#include <algorithm>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <vector>
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

template <typename T> void shuffleRowsIn2DVector(std::vector<std::vector<T>> &twoDVector) {
  // Create a vector of indices
  std::vector<size_t> indices(twoDVector.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }

  // Shuffle the indices using std::shuffle
  std::random_device rd;
  std::mt19937 rng(rd()); // Use a random number generator
  std::shuffle(indices.begin(), indices.end(), rng);

  // Create a temporary vector to hold the shuffled 2D vector
  std::vector<std::vector<T>> shuffledTwoDVector(twoDVector.size());

  // Populate the temporary vector with shuffled rows
  for (size_t i = 0; i < indices.size(); ++i) {
    shuffledTwoDVector[i] = twoDVector[indices[i]];
  }
  // Replace the original 2D vector with the shuffled vector
  twoDVector = std::move(shuffledTwoDVector);
}

class RandomChainsGenerator {
public:
  RandomChainsGenerator() {}
  RandomChainsGenerator(const DAG_Model &dag_tasks) : dag_tasks_(dag_tasks) {}

  std::vector<std::vector<int>> GenerateChainType3(int chain_num) {
    std::vector<std::vector<int>> chains;
    chains.reserve(chain_num);
    for (int source_id = 0; source_id < dag_tasks_.tasks.size(); source_id++) {
      for (int sink_id = 0; sink_id < dag_tasks_.tasks.size(); sink_id++) {
        if (source_id == sink_id)
          continue;
        std::vector<int> path = shortest_paths(source_id, sink_id, dag_tasks_.graph_);
        if (path.size() > 1) {
          PushChain(path);
        }
      }
    }
  }

  size_t CountPeriodType(const std::vector<int> &chain) {
    std::unordered_set<int> set;
    for (int task_id : chain) {
      int period_curr = dag_tasks_.tasks[task_id].period;
      set.insert(period_curr);
    }
    return set.size();
  }

  void PushChain(const std::vector<int> &chain) {
    size_t period_count = CountPeriodType(chain);
    if (period_count > 3)
      return;
    chains_all_period_[period_count - 1].push_back(chain);
  }

  std::vector<std::vector<int>> ObtainRandomChains(int chain_num) {

    std::vector<std::vector<int>> chains;
    std::vector<double> chain_dist = {0.7, 0.2, 0.1};
    for (int period_type = 1; period_type <= 3; period_type++) {
      std::vector<std::vector<int>> &chains_curr = chains_all_period_[period_type - 1];
      shuffleRowsIn2DVector<int>(chains_curr);
      chains_curr.resize(round(chain_num * chain_dist[period_type - 1]));
      chains.insert(chains.end(), chains_curr.begin(), chains_curr.end());
    }
    shuffleRowsIn2DVector<int>(chains);
    return chains;
  }

  // data members
  DAG_Model dag_tasks_;
  // chains with different types of periods,
  // [0] -> only 1 type of period,
  // [1] -> 2 types of periods,
  // [2] -> 3 types of periods
  std::vector<std::vector<std::vector<int>>> chains_all_period_;
};

TEST(RandomChainsGenerator, genreate_random_chains) { ; }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}