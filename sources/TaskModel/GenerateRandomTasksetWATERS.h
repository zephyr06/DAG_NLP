

#pragma once

#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/GenerateRandomTaskset.h"

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
  RandomChainsGenerator(const DAG_Model &dag_tasks) : dag_tasks_(dag_tasks) {
    for (int i = 0; i < 3; i++) {
      chains_all_period_.push_back({});
    }
  }

  std::vector<std::vector<int>> GenerateChain(int chain_num) {
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
    return ObtainRandomChains(chain_num);
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
      int target_size = round(chain_num * chain_dist[period_type - 1]);
      if (chains_curr.size() >= target_size)
        chains_curr.resize(target_size);
      else
        return {};
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

void AddChains2DAG(DAG_Model &dag_tasks, int numCauseEffectChain);

DAG_Model GenerateDAG_WATERS15(int N, double totalUtilization, int numberOfProcessor, int periodMin,
                               int periodMax, int coreRequireMax, int sf_fork_num, int fork_sensor_num_min,
                               int fork_sensor_num_max, int numCauseEffectChain, int taskSetType,
                               int deadlineType);
