#include "sources/Baseline/SimulatedAnnealing/OptimizeSA.h"

namespace OrderOptDAG_SPACE {
using namespace OptimizeSF;

std::vector<std::pair<int, int>> GetJobMinMaxStartTimeRange(const TaskSetInfoDerived &tasks_info) {
  std::vector<std::pair<int, int>> job_min_max_start_time_range;
  job_min_max_start_time_range.reserve(tasks_info.variableDimension);
  for (int i = 0; i < tasks_info.N; i++) {
    for (int j = 0; j < tasks_info.hyper_period / tasks_info.tasks[i].period; j++) {
      job_min_max_start_time_range.push_back(
          std::make_pair(j * tasks_info.tasks[i].period,
                         (j + 1) * tasks_info.tasks[i].period - tasks_info.tasks[i].executionTime));
    }
  }
  return job_min_max_start_time_range;
}

} // namespace OrderOptDAG_SPACE
