#pragma once
#include <bits/stdc++.h>

#include <algorithm> // for std::for_each
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_list.hpp>       // adjacency_list
#include <boost/graph/breadth_first_search.hpp> // shortest paths
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graphviz.hpp> // read_graphviz
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/topological_sort.hpp> // find_if
#include <boost/range/algorithm.hpp>        // range find_if
#include <boost/utility.hpp>                // for boost::tie
#include <iostream>                         // for std::cout
#include <utility>
#include <utility> // for std::pair

#include "sources/TaskModel/Edge.h"
#include "sources/TaskModel/RegularTasks.h"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                              boost::property<boost::vertex_name_t, LLint>,
                              boost::property<boost::edge_name_t, LLint>>
    Graph;
// map to access properties of vertex from the graph
typedef boost::property_map<Graph, boost::vertex_name_t>::type vertex_name_map_t;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::property_map<Graph, boost::edge_name_t>::type edge_name_map_t;

typedef std::unordered_map<LLint, Vertex> indexVertexMap;

struct first_name_t {
  typedef boost::vertex_property_tag kind;
};

namespace OrderOptDAG_SPACE {
// Code from
// https://stackoverflow.com/questions/52878925/boostgraph-getting-the-path-up-to-the-root
static constexpr Vertex NIL = -1;
std::vector<int> shortest_paths(Vertex root, Vertex target, const Graph &g);

struct SF_Fork {
  SF_Fork() {}
  SF_Fork(const std::vector<int> &source, int sink) : source(source), sink(sink) {}

  // data member
  std::vector<int> source;
  int sink;
};

void PrintChains(const std::vector<std::vector<int>> &chains);
// *2, 1 means task 2 depend on task 1, or task 1 must execute before task 2;
// 1 would be the first in MAP_Prev, while 2 is one task in TaskSet
// MAP_Prev maps one task to all the tasks it depends on
typedef std::map<int, RegularTaskSystem::TaskSet> MAP_Prev;
using namespace RegularTaskSystem;
class DAG_Model {
public:
  DAG_Model() {}
  DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev, int num_fork, int fork_sensor_num_min, int fork_sensor_num_max,
            int numCauseEffectChain = 1)
      : tasks(tasks), mapPrev(mapPrev) {
    ConstructBGL_Graph();
    chains_ = GetRandomChains(numCauseEffectChain);
    sf_forks_ = GetRandomForks(num_fork, fork_sensor_num_min, fork_sensor_num_max);
  }

  DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev, int numCauseEffectChain)
      : DAG_Model(tasks, mapPrev, 0, 0, 0, numCauseEffectChain) {}

  std::pair<Graph, indexVertexMap> GenerateGraphForTaskSet() const;

  void addEdge(int prevIndex, int nextIndex) { mapPrev[nextIndex].push_back(tasks[prevIndex]); }

  void print();

  void printChains();

  inline void ConstructBGL_Graph() { std::tie(graph_, indexesBGL_) = GenerateGraphForTaskSet(); }

  TaskSet GetTasks() const { return tasks; }

  int edgeNumber();
  std::vector<SF_Fork> GetRandomForks(int num_fork, int fork_sensor_num_min, int fork_sensor_num_max);

  std::vector<std::vector<int>> GetRandomChains(int numOfChains);
  void SetChains(std::vector<std::vector<int>> &chains) { chains_ = chains; }
  std::vector<int> FindSourceTaskIds() const;

  std::vector<int> FindSinkTaskIds() const;

  // data members

public:
  TaskSet tasks;
  MAP_Prev mapPrev;
  Graph graph_;
  indexVertexMap indexesBGL_;
  std::vector<std::vector<int>> chains_;
  std::vector<SF_Fork> sf_forks_;
};

DAG_Model ReadDAG_Tasks(std::string path, std::string priorityType = "orig", int chainNum = 100);

std::vector<std::vector<int>> GetChainsForSF(const DAG_Model &dag_tasks);
} // namespace OrderOptDAG_SPACE