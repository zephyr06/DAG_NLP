#pragma once
#include <dirent.h>
#include <sys/types.h>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/BatchUtils.h"
#include <chrono>
#include <fstream>
#include <iostream>

namespace OrderOptDAG_SPACE {

void UpdateActivationPattern(const std::vector<int> &chain, const DAG_Model &dagTasks,
                             std::vector<int> &activation_pattern_count) {
  std::unordered_set<int> period_set;
  for (int task_id : chain) {
    period_set.insert(dagTasks.tasks[task_id].period);
  }
  activation_pattern_count[period_set.size() - 1]++;
}

void UpdateRunnableCount(const std::vector<int> &chain, const DAG_Model &dagTasks,
                         std::vector<int> &activation_pattern_count) {
  std::unordered_map<int, int> pattern2task_count;
  for (int task_id : chain) {
    int period = dagTasks.tasks[task_id].period;
    if (pattern2task_count.count(period) > 0)
      pattern2task_count[period]++;
    else
      pattern2task_count[period] = 1;
  }
  for (auto pair : pattern2task_count) {
    activation_pattern_count[pair.second - 1]++;
  }
}

void AnalyzeTaskSet(std::string dataSetFolder = GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/dagTasks/") {
  std::string dirStr = dataSetFolder;
  const char *pathDataset = (dirStr).c_str();
  std::cout << "Dataset Directory: " << pathDataset << std::endl;

  // Prepare intermediate data records
  std::vector<int> activation_pattern_count(3); // involved activation patterns are 1, 2, 3
  std::vector<int> runnable_per_pattern(5);     // number of runnables per pattern are 1 2 3 4 5

  std::vector<std::string> files = ReadFilesInDirectory(pathDataset);
  for (const auto &file : files) {
    std::string delimiter = "-";
    if (file.substr(0, file.find(delimiter)) == "dag" && file.find("Res") == std::string::npos &&
        file.find("LoopCount") == std::string::npos) {
      std::cout << file << std::endl;
      std::string path = dataSetFolder + file;
      OrderOptDAG_SPACE::DAG_Model dagTasks =
          OrderOptDAG_SPACE::ReadDAG_Tasks(path, GlobalVariablesDAGOpt::priorityMode);

      std::vector<std::vector<int>> chains = dagTasks.chains_;
      for (const auto &chain : chains) {
        UpdateActivationPattern(chain, dagTasks, activation_pattern_count);
        UpdateRunnableCount(chain, dagTasks, runnable_per_pattern);
      }
    }
  }

  // print results
  double total_pattern_count =
      std::accumulate(activation_pattern_count.begin(), activation_pattern_count.end(), 0);
  std::cout << "Activation pattern proportion: \n"
            << "1: " << activation_pattern_count[0] / total_pattern_count << "\n"
            << "2: " << activation_pattern_count[1] / total_pattern_count << "\n"
            << "3: " << activation_pattern_count[2] / total_pattern_count << "\n";

  double total_runnable = std::accumulate(runnable_per_pattern.begin(), runnable_per_pattern.end(), 0);
  std::cout << "Runnable proportion: \n"
            << "1: " << runnable_per_pattern[0] / total_runnable << "\n"
            << "2: " << runnable_per_pattern[1] / total_runnable << "\n"
            << "3: " << runnable_per_pattern[2] / total_runnable << "\n"
            << "4: " << runnable_per_pattern[3] / total_runnable << "\n"
            << "5: " << runnable_per_pattern[4] / total_runnable << "\n";
}
} // namespace OrderOptDAG_SPACE