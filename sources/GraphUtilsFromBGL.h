#pragma once
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

#include "sources/RegularTasks.h"
#include "sources/DeclareDAG.h"
#include "sources/DAG_Model.h"
namespace RTSS21IC_NLP
{

    using namespace std;
    using namespace boost;

    typedef adjacency_list<vecS, vecS, bidirectionalS,
                           property<vertex_name_t, LLint>,
                           property<edge_name_t, LLint>>
        Graph;
    typedef property_map<Graph, vertex_name_t>::type vertex_name_map_t;
    typedef graph_traits<Graph>::vertex_descriptor Vertex;
    typedef property_map<Graph, edge_name_t>::type edge_name_map_t;

    typedef std::unordered_map<LLint, Vertex> indexVertexMap;

    struct first_name_t
    {
        typedef boost::vertex_property_tag kind;
    };

    pair<Graph, indexVertexMap> EstablishGraphStartTimeVector(DAG_SPACE::DAG_Model &dagTasks)
    {
        using namespace DAG_SPACE;
        TaskSet tasks = dagTasks.tasks;
        int N = tasks.size();
        LLint hyperPeriod = HyperPeriod(tasks);

        // declare variables
        vector<LLint> sizeOfVariables;
        int variableDimension = 0;
        for (int i = 0; i < N; i++)
        {
            LLint size = hyperPeriod / tasks[i].period;
            sizeOfVariables.push_back(size);
            variableDimension += size;
        }
        // auto actual = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);

        Graph g;

        // map to access properties of vertex from the graph
        vertex_name_map_t vertex2indexBig = get(vertex_name, g);

        // map to access vertex from its global index

        indexVertexMap indexesBGL;
        for (LLint i = 0; i < variableDimension; i++)
        {
            indexVertexMap::iterator pos;
            bool inserted;
            Vertex u;
            boost::tie(pos, inserted) = indexesBGL.insert(std::make_pair(i, Vertex()));
            if (inserted)
            {
                // u = pos->second;
                u = add_vertex(g);
                vertex2indexBig[u] = i;
                // make sure the inserted vertex in indexesBGL
                //  is the same as the one inserted in the graph
                pos->second = u;
            }
            else
            {
                CoutError("Error building indexVertexMap!");
            }
        }
        // graph_traits<Graph>::vertex_iterator i, end;
        // for (boost::tie(i, end) = vertices(g); i != end; ++i)
        // {
        //     std::cout << vertex2indexBig[*i] << std::endl;
        // }
        return make_pair(g, indexesBGL);
    }

    void FindSubTree(Graph &g, vector<LLint> &subTreeIndex, std::unordered_set<int> &indexSet, Vertex v)
    {
        vertex_name_map_t vertex2indexBig = get(vertex_name, g);

        if (indexSet.find(vertex2indexBig[v]) == indexSet.end())
        {
            indexSet.insert(vertex2indexBig[v]);
            subTreeIndex.push_back(vertex2indexBig[v]);
        }

        boost::graph_traits<Graph>::out_edge_iterator eo, edge_end_o;
        for (boost::tie(eo, edge_end_o) = out_edges(v, g); eo != edge_end_o; ++eo)
        {
            Vertex vvv = target(*eo, g);
            if (indexSet.find(vertex2indexBig[vvv]) == indexSet.end())
            {
                subTreeIndex.push_back(vertex2indexBig[vvv]);
                indexSet.insert(vertex2indexBig[vvv]);
                FindSubTree(g, subTreeIndex, indexSet, vvv);
            }
            else
                continue;
        }

        boost::graph_traits<Graph>::in_edge_iterator ei, edge_end;
        for (boost::tie(ei, edge_end) = in_edges(v, g); ei != edge_end; ++ei)
        {
            Vertex vv = source(*ei, g);
            if (indexSet.find(vertex2indexBig[vv]) == indexSet.end())
            {
                subTreeIndex.push_back(vertex2indexBig[vv]);
                indexSet.insert(vertex2indexBig[vv]);
                FindSubTree(g, subTreeIndex, indexSet, vv);
            }
            else
                continue;
        }
        return;
    }

    void FindSubTree(Graph &g, vector<LLint> &subTreeIndex, Vertex v)
    {
        std::unordered_set<int> indexSet;
        FindSubTree(g, subTreeIndex, indexSet, v);
    }

    // template <class EdgeIter, class Graph>
    // void who_owes_who(EdgeIter first, EdgeIter last, const Graph &G)
    // {
    //     // Access the propety acessor type for this graph
    //     typedef typename property_map<Graph,
    //                                   first_name_t>::const_type NameMap;
    //     NameMap name = get(first_name_t(), G);

    //     typedef typename boost::property_traits<NameMap>::value_type NameType;

    //     NameType src_name, targ_name;

    //     while (first != last)
    //     {
    //         src_name = boost::get(name, source(*first, G));
    //         targ_name = boost::get(name, target(*first, G));
    //         // cout << src_name << " owes "
    //         //      << targ_name << " some money" << endl;
    //         cout << src_name.id << ", " << targ_name.id << endl;
    //         ++first;
    //     }
    // }

    /**
     * @brief
     *
     * @param dagTasks
     * @return vector<int> execution order of task index, first task first
     */
    vector<int> FindDependencyOrder(const DAG_SPACE::DAG_Model &dagTasks)
    {

        using namespace RegularTaskSystem;
        using namespace DAG_SPACE;
        int N = dagTasks.tasks.size();

        typedef boost::property<first_name_t, Task> FirstNameProperty;
        typedef boost::adjacency_list<vecS, vecS, bidirectionalS, FirstNameProperty> Graph;
        typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
        typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

        Graph g(N);

        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            size_t indexNext = itr->first;
            for (size_t i = 0; i < tasksPrev.size(); i++)
            {
                boost::add_edge(tasksPrev[i].id, dagTasks.tasks[indexNext].id, g);
            }
        }
        boost::property_map<Graph, first_name_t>::type
            name = get(first_name_t(), g);
        for (int i = 0; i < int(dagTasks.tasks.size()); i++)
            boost::put(name, i, dagTasks.tasks[i]);
        // who_owes_who(edges(g).first, edges(g).second, g);

        typedef std::list<vertex_t> MakeOrder;
        MakeOrder make_order;
        boost::topological_sort(g, std::front_inserter(make_order));

        // std::cout << "dependency ordering: ";
        vector<int> executionOrder;
        executionOrder.reserve(N);
        for (MakeOrder::iterator i = make_order.begin();
             i != make_order.end(); ++i)
        {
            executionOrder.push_back(*i);
            // cout << *i << endl;
        }
        return executionOrder;
    }

    /**
     * @brief return the index of sink node
     *
     * @param dagTasks
     * @return int
     */
    int FindSinkNode(DAG_SPACE::DAG_Model dagTasks)
    {

        using namespace RegularTaskSystem;
        using namespace DAG_SPACE;
        int N = dagTasks.tasks.size();

        typedef boost::property<first_name_t, Task> FirstNameProperty;
        typedef boost::adjacency_list<vecS, vecS, bidirectionalS, FirstNameProperty> Graph;
        typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
        typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

        Graph g(N);

        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            size_t indexNext = itr->first;
            for (size_t i = 0; i < tasksPrev.size(); i++)
            {
                boost::add_edge(tasksPrev[i].id, dagTasks.tasks[indexNext].id, g);
            }
        }
        boost::property_map<Graph, first_name_t>::type
            name = get(first_name_t(), g);
        for (int i = 0; i < 5; i++)
            boost::put(name, i, dagTasks.tasks[i]);
        // who_owes_who(edges(g).first, edges(g).second, g);

        typedef std::list<vertex_t> MakeOrder;
        MakeOrder make_order;
        boost::topological_sort(g, std::front_inserter(make_order));

        // std::cout << "dependency ordering: ";
        MakeOrder::iterator i = make_order.end();
        i--;

        return *i;
    }

} // namespace RTSS21IC_NLP
