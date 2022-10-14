#pragma once
#include "sources/TaskModel/RegularTasks.h"
#include <utility>
#include <boost/config.hpp>
#include <iostream>  // for std::cout
#include <utility>   // for std::pair
#include <algorithm> // for std::for_each
#include <bits/stdc++.h>
#include <boost/utility.hpp> // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/topological_sort.hpp>

#include <boost/graph/adjacency_list.hpp>       // adjacency_list
#include <boost/graph/topological_sort.hpp>     // find_if
#include <boost/graph/breadth_first_search.hpp> // shortest paths
#include <boost/range/algorithm.hpp>            // range find_if
#include <boost/graph/graphviz.hpp>             // read_graphviz

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                              boost::property<boost::vertex_name_t, LLint>,
                              boost::property<boost::edge_name_t, LLint>>
    Graph;
// map to access properties of vertex from the graph
typedef boost::property_map<Graph, boost::vertex_name_t>::type vertex_name_map_t;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::property_map<Graph, boost::edge_name_t>::type edge_name_map_t;

typedef std::unordered_map<LLint, Vertex> indexVertexMap;

struct first_name_t
{
    typedef boost::vertex_property_tag kind;
};

// Code from https://stackoverflow.com/questions/52878925/boostgraph-getting-the-path-up-to-the-root
static constexpr Vertex NIL = -1;
std::vector<int> shortest_paths(Vertex root, Vertex target, Graph const &g)
{
    std::vector<int> path;
    // find shortest paths from the root
    std::vector<Vertex> predecessors(boost::num_vertices(g), NIL);
    auto recorder = boost::record_predecessors(predecessors.data(), boost::on_examine_edge());
    boost::breadth_first_search(g, root, boost::visitor(boost::make_bfs_visitor(recorder)));

    for (auto pred = predecessors[target]; pred != NIL; pred = predecessors[pred])
    {
        path.push_back(pred);
    }
    if (path.size() != 0)
    {
        std::reverse(path.begin(), path.end());
        path.push_back(target);
    }
    return path;
}

void PrintChains(std::vector<std::vector<int>> &chains)
{
    std::cout << "Chains:" << std::endl;
    for (size_t i = 0; i < size(chains); i++)
    {
        for (size_t j = 0; j < size(chains[i]); j++)
        {
            std::cout << chains[i][j] << ", ";
        }
        std::cout << endl;
    }
}

namespace OrderOptDAG_SPACE
{
    // *2, 1 means task 2 depend on task 1, or task 1 must execute before task 2;
    // 1 would be the first in MAP_Prev, while 2 is one task in TaskSet
    // MAP_Prev maps one task to all the tasks it depends on
    typedef std::map<int, RegularTaskSystem::TaskSet> MAP_Prev;
    using namespace RegularTaskSystem;
    class DAG_Model
    {
    public:
        TaskSet tasks;
        MAP_Prev mapPrev;
        Graph graph_;
        indexVertexMap indexesBGL_;
        std::vector<std::vector<int>> chains_;

        DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev) : tasks(tasks),
                                                       mapPrev(mapPrev)
        {
            std::tie(graph_, indexesBGL_) = GenerateGraphForTaskSet();
            chains_ = GetRandomChains(NumCauseEffectChain);
        }

        pair<Graph, indexVertexMap> GenerateGraphForTaskSet()
        {

            Graph g;
            // map to access properties of vertex from the graph
            vertex_name_map_t vertex2indexBig = get(boost::vertex_name, g);

            // map to access vertex from its global index
            indexVertexMap indexesBGL;
            for (uint i = 0; i < tasks.size(); i++)
            {
                indexVertexMap::iterator pos;
                bool inserted;
                Vertex u;
                boost::tie(pos, inserted) = indexesBGL.insert(std::make_pair(i, Vertex()));
                if (inserted)
                {
                    u = add_vertex(g);
                    vertex2indexBig[u] = i;
                    pos->second = u;
                }
                else
                {
                    CoutError("Error building indexVertexMap!");
                }
            }

            // add edges

            for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
            {
                const TaskSet &tasksPrev = itr->second;
                size_t indexNext = itr->first;
                for (size_t i = 0; i < tasksPrev.size(); i++)
                {
                    boost::add_edge(tasksPrev[i].id, tasks[indexNext].id, g);
                }
            }
            return std::make_pair(g, indexesBGL);
        }

        void addEdge(int prevIndex, int nextIndex)
        {
            mapPrev[nextIndex].push_back(tasks[prevIndex]);
        }

        void print()
        {
            for (auto &task : tasks)
                task.print();
            for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
            {
                for (uint i = 0; i < (itr->second).size(); i++)
                    cout << "Edge: " << ((itr->second)[i].id) << "-->" << (itr->first) << endl;
            }
        }

        void printChains()
        {
            for (size_t i = 0; i < chains_.size(); i++)
            {
                std::cout << "Chain #" << i << ": ";
                for (auto task : chains_[i])
                {
                    std::cout << task << ", ";
                }
                std::cout << std::endl;
            }
        }

        TaskSet GetTasks() const
        {
            return tasks;
        }

        int edgeNumber()
        {
            int count = 0;
            for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
            {
                count += (itr->second).size();
            }
            return count;
        }

        std::vector<std::vector<int>> GetRandomChains(int numOfChains)
        {
            std::vector<std::vector<int>> chains;
            chains.reserve(numOfChains);
            int chainCount = 0;
            std::vector<int> sourceIds = FindSourceTaskIds();
            std::vector<int> sinkIds = FindSinkTaskIds();

            for (int sourceId : sourceIds)
            {
                for (int sinkId : sinkIds)
                {
                    auto path = shortest_paths(sourceId, sinkId, graph_);
                    if (path.size() > 1)
                    {
                        chains.push_back(path);
                        chainCount++;
                    }
                }
            }
            if (chainCount > numOfChains)
            {
                if (whether_shuffle_CE_chain)
                    std::shuffle(chains.begin(), chains.end(), std::default_random_engine{});
                chains.resize(numOfChains);
            }
            return chains;
        }
        void SetChains(std::vector<std::vector<int>> &chains)
        {
            chains_ = chains;
        }
        std::vector<int> FindSourceTaskIds() const
        {
            std::set<int> originTasks;
            for (uint i = 0; i < tasks.size(); i++)
            {
                originTasks.insert(i);
            }

            for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
            {
                size_t indexNext = itr->first;
                originTasks.erase(indexNext);
            }
            std::vector<int> res(originTasks.size());
            std::copy(originTasks.begin(), originTasks.end(), res.begin());
            return res;
        }

        std::vector<int> FindSinkTaskIds() const
        {
            std::set<int> originTasks;
            for (uint i = 0; i < tasks.size(); i++)
            {
                originTasks.insert(i);
            }

            for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
            {
                auto parents = itr->second;
                for (auto p : parents)
                {
                    originTasks.erase(p.id);
                }
            }
            std::vector<int> res(originTasks.size());
            std::copy(originTasks.begin(), originTasks.end(), res.begin());
            return res;
        }
    };
    // it seems like only 'orig' priority type is allowed
    DAG_Model ReadDAG_Tasks(string path, string priorityType = "orig")
    {
        TaskSet tasks = ReadTaskSet(path, priorityType);
        // some default parameters in this function
        string delimiter = ",";
        string token;
        string line;
        size_t pos = 0;

        MAP_Prev mapPrev;

        fstream file;
        file.open(path, ios::in);
        if (file.is_open())
        {
            string line;
            while (getline(file, line))
            {
                if (line[0] != '*')
                    continue;
                line = line.substr(1, int(line.size()) - 1);
                vector<int> dataInLine;
                while ((pos = line.find(delimiter)) != string::npos)
                {
                    token = line.substr(0, pos);
                    int temp = atoi(token.c_str());
                    dataInLine.push_back(temp);
                    line.erase(0, pos + delimiter.length());
                }
                dataInLine.push_back(atoi(line.c_str()));
                mapPrev[dataInLine[1]].push_back(tasks[dataInLine[0]]);
            }

            DAG_Model ttt(tasks, mapPrev);
            return ttt;
        }
        else
        {
            cout << Color::red << "The path does not exist in ReadTaskSet!" << endl
                 << path
                 << Color::def << endl;
            throw;
        }
    }
}