#pragma once
#include "unordered_map"
#include "set"

#include "sources/TaskModel/DAG_Model.h"

namespace OrderOptDAG_SPACE
{
  /**
   * @brief
   *
   * @param dagTasks
   * @returnstd::vector<int> execution order of task index, first task first
   */
  std::vector<int> FindDependencyOrderDFS(const OrderOptDAG_SPACE::DAG_Model &dagTasks);

  TaskSet FindSourceTasks(const DAG_Model &dagTasks);
  void AddNodeTS(int taskId, const DAG_Model &dagTasks, std::vector<int> &path, std::vector<bool> &visited, Graph &graphBoost, indexVertexMap &indexesBGL);

  std::vector<int> TopologicalSortSingle(TaskSet &originTasks, const DAG_Model &dagTasks, Graph &graphBoost, indexVertexMap &indexesBGL);

  std::vector<std::vector<int>> TopologicalSortMulti( DAG_Model dagTasks);
} // namespace OrderOptDAG_SPACE