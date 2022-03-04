#pragma once
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

#include "../sources/RegularTasks.h"
#include "../sources/DeclareDAG.h"
#include "../sources/DAG_Model.h"

using namespace std;
using namespace boost;
using namespace RegularTaskSystem;

typedef adjacency_list<vecS, vecS, bidirectionalS,
                       property<vertex_name_t, LLint>,
                       property<edge_name_t, LLint>>
    Graph;
// map to access properties of vertex from the graph
typedef property_map<Graph, vertex_name_t>::type vertex_name_map_t;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef property_map<Graph, edge_name_t>::type edge_name_map_t;

typedef std::unordered_map<LLint, Vertex> indexVertexMap;

struct first_name_t
{
    typedef boost::vertex_property_tag kind;
};

pair<Graph, indexVertexMap> EstablishGraphStartTimeVector(RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
{
    using namespace DAG_SPACE;

    Graph g;
    // map to access properties of vertex from the graph
    vertex_name_map_t vertex2indexBig = get(vertex_name, g);

    // map to access vertex from its global index

    indexVertexMap indexesBGL;
    for (LLint i = 0; i < tasksInfo.variableDimension; i++)
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
    return std::make_pair(g, indexesBGL);
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

/**
     * @brief m maps from index in original startTimeVector to index in compressed startTimeVector
     * 
     * @param maskForEliminate 
     * @return std::unordered_map<LLint, LLint> 
     */
std::unordered_map<LLint, LLint> MapIndex_True2Compress(const vector<bool> &maskForEliminate)
{

    std::unordered_map<LLint, LLint> m;
    // count is the index in compressed startTimeVector
    int count = 0;
    for (size_t i = 0; i < maskForEliminate.size(); i++)
    {
        if (maskForEliminate.at(i) == false)
            m[i] = count++;
    }
    return m;
}

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
    // typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

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
    // typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

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

struct EliminationForest
{
    MAP_Index2Data mapIndex;
    vector<bool> maskForEliminate;
    Graph eliminationTrees;
    indexVertexMap indexesBGL;
    std::unordered_map<LLint, LLint> mapIndex_True2Compress;
    LLint lengthCompressed;

    /**
     * @brief Construct a new Elimination Forest object:
     *  all the elements are initialized with no dependency relationship
     * 
     * @param tasksInfo 
     */
    EliminationForest(RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        maskForEliminate = vector<bool>(tasksInfo.variableDimension, false);

        for (LLint i = 0; i < tasksInfo.variableDimension; i++)
        {
            MappingDataStruct m{i, 0};
            mapIndex[i] = m;
        };

        pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(tasksInfo);
        eliminationTrees = sth.first;
        indexesBGL = sth.second;

        mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);

        lengthCompressed = 0;
        for (LLint i = 0; i < tasksInfo.length; i++)
        {
            if (maskForEliminate[i] == false)
                lengthCompressed++;
        }
    }

    /**
     * @brief add a dependency relationship to mapIndex, 
     * after elimination, we have:
     * dependent = x + distance
     * 
     * @param dependent 
     * @param x 
     * @param distance 
     */
    void AddLinearEliminate(LLint dependent, LLint x, double distance)
    {
        maskForEliminate[dependent] = true;
        mapIndex[dependent] = MappingDataStruct{x, distance};
        mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);
        lengthCompressed--;

        // add edge to eliminationTrees
        graph_traits<Graph>::edge_descriptor e;
        bool inserted;
        boost::tie(e, inserted) = add_edge(indexesBGL[dependent],
                                           indexesBGL[x],
                                           eliminationTrees);
    }
};
VectorDynamic RecoverStartTimeVector(const VectorDynamic &compressed,
                                     const vector<bool> &maskForEliminate,
                                     const MAP_Index2Data &mapIndex)
{
    LLint variableDimension = maskForEliminate.size();
    vector<bool> filledTable(variableDimension, 0);

    VectorDynamic actual = GenerateVectorDynamic(variableDimension);
    LLint index = 0;
    for (size_t i = 0; i < (size_t)variableDimension; i++)
    {
        if (not maskForEliminate.at(i))
        {
            filledTable[i] = 1;
            actual[i] = compressed(index++, 0);
        }
    }
    for (size_t i = 0; i < (size_t)variableDimension; i++)
    {
        if (not filledTable[i])
        {
            actual(i, 0) = GetSingleElement(i, actual, mapIndex, filledTable);
        }
    }
    return actual;
}
VectorDynamic RecoverStartTimeVector(const VectorDynamic &compressed,
                                     const EliminationForest &forestInfo)
{
    return RecoverStartTimeVector(compressed, forestInfo.maskForEliminate, forestInfo.mapIndex);
}
/**
     * @brief Given an index, find the final index that it depends on;
     * 
     * @param index 
     * @param mapIndex 
     * @return LLint 
     */
LLint FindLeaf(LLint index, const MAP_Index2Data &mapIndex)
{
    if (index == mapIndex.at(index).getIndex())
        return index;
    else
        return FindLeaf(mapIndex.at(index).getIndex(), mapIndex);
    return -1;
}

/**
     * @brief generate analytic Jacobian for elimination part
     * 
     * a sparse matrix that represents Jacobian matrix of compreseed variables w.r.t. original variables 
     */
// SM_Dynamic JacobianElimination(LLint length, LLint lengthCompressed,
//                                const vector<LLint> &sizeOfVariables,
//                                const MAP_Index2Data &mapIndex,
//                                const std::unordered_map<LLint, LLint> &mapIndex_True2Compress)
SM_Dynamic JacobianElimination(const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                               const EliminationForest &forestInfo)
{
    SM_Dynamic j_map2(tasksInfo.variableDimension, forestInfo.lengthCompressed);
    SM_Dynamic j_map(tasksInfo.variableDimension, tasksInfo.variableDimension);
    int countEliminatedVariable = 0;
    // go through all the variables
    for (int i = 0; i < int(tasksInfo.sizeOfVariables.size()); i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables.at(i)); j++)
        {
            // if()
            LLint bigIndex = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            // find its final dependency variable
            LLint finalIndex = FindLeaf(bigIndex, forestInfo.mapIndex);
            // j_map.insert(mapIndex_True2Compress.at(finalIndex), bigIndex - countEliminatedVariable) = 1;
            j_map.insert(bigIndex, finalIndex) = 1;
            if (forestInfo.mapIndex.at(bigIndex).getIndex() != bigIndex)
                countEliminatedVariable++;
            else
                j_map2.insert(bigIndex, finalIndex - countEliminatedVariable) = 1;
        }
    }
    return j_map * j_map2;
}