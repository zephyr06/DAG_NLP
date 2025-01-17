#include "sources/Optimization/TopologicalSort.h"
namespace OrderOptDAG_SPACE
{
   std::vector<int> FindDependencyOrderDFS(const OrderOptDAG_SPACE::DAG_Model &dagTasks)
    {

        using namespace RegularTaskSystem;
        using namespace OrderOptDAG_SPACE;
        int N = dagTasks.tasks.size();

        typedef boost::property<first_name_t, Task> FirstNameProperty;
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, FirstNameProperty> Graph;

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
            name = boost::get(first_name_t(), g);
        for (int i = 0; i < int(dagTasks.tasks.size()); i++)
            boost::put(name, i, dagTasks.tasks[i]);
        // who_owes_who(edges(g).first, edges(g).second, g);

        typedef std::list<Vertex> MakeOrder;
        MakeOrder make_order;
        boost::topological_sort(g, std::front_inserter(make_order));

        // std::cout << "dependency ordering: ";
       std::vector<int> executionOrder;
        executionOrder.reserve(N);
        for (MakeOrder::iterator i = make_order.begin();
             i != make_order.end(); ++i)
        {
            executionOrder.push_back(*i);
            //std::cout << *i <<std::endl;
        }
        return executionOrder;
    }

    TaskSet FindSourceTasks(const DAG_Model &dagTasks)
    {
        std::vector<int> originTasks = dagTasks.FindSourceTaskIds();

        TaskSet tasks;
        for (auto itr = originTasks.begin(); itr != originTasks.end(); itr++)
        {
            tasks.push_back(dagTasks.tasks[*itr]);
        }
        return tasks;
    }

    void AddNodeTS(int taskId, const DAG_Model &dagTasks, std::vector<int> &path, std::vector<bool> &visited, Graph &graphBoost, indexVertexMap &indexesBGL)
    {
        if (visited[taskId])
        {
            return;
        }

        const MAP_Prev &mapPrev = dagTasks.mapPrev;
        uint dependentTasks = 0;
        if (mapPrev.find(taskId) != mapPrev.end())
        {
            dependentTasks = dagTasks.mapPrev.at(taskId).size();
            // add its previous tasks
            for (uint i = 0; i < dependentTasks; i++)
            {
                AddNodeTS(dagTasks.mapPrev.at(taskId)[i].id, dagTasks, path, visited, graphBoost, indexesBGL);
            }
        }
        // all precedence nodes, if any, have already been added
        if (!visited[taskId])
        {
            path.push_back(taskId);
            visited[taskId] = true;
        }

        // add its following tasks
        Vertex v = indexesBGL[taskId];
        boost::graph_traits<Graph>::out_edge_iterator eo,
            edge_end_o;
        vertex_name_map_t vertex2indexBig = get(boost::vertex_name, graphBoost);
        for (boost::tie(eo, edge_end_o) = boost::out_edges(v, graphBoost); eo != edge_end_o; ++eo)
        {
            Vertex vvv = target(*eo, graphBoost);
            AddNodeTS(vertex2indexBig[vvv], dagTasks, path, visited, graphBoost, indexesBGL);
        }
    }

    std::vector<int> TopologicalSortSingle(TaskSet &originTasks, const DAG_Model &dagTasks, Graph &graphBoost, indexVertexMap &indexesBGL)
    {
        std::vector<int> path;
        path.reserve(dagTasks.tasks.size());
        std::vector<bool> visited(dagTasks.tasks.size(), false);
        for (size_t i = 0; i < originTasks.size(); i++)
        {
            int sourceId = originTasks[i].id;
            AddNodeTS(sourceId, dagTasks, path, visited, graphBoost, indexesBGL);
        }
        if (path.size() != dagTasks.tasks.size())
        {
            CoutError("TopologicalSortSingle failed!");
        }
        return path;
    }

    std::vector<std::vector<int>> TopologicalSortMulti( DAG_Model dagTasks)
    {
        std::vector<std::string> priorityTypeList = {"RM", "utilization", "DM", "orig"};
        std::vector<std::vector<int>> paths;
        paths.reserve(priorityTypeList.size());

        Graph graphBoost;
        indexVertexMap indexesBGL;
        std::tie(graphBoost, indexesBGL) = dagTasks.GenerateGraphForTaskSet();

        TaskSet originTasks = FindSourceTasks(dagTasks);

        for (std::string priorityType : priorityTypeList)
        {
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                itr->second = Reorder(itr->second, priorityType);
            }

            originTasks = Reorder(originTasks, priorityType);
            std::vector<int> topoOrder = TopologicalSortSingle(originTasks, dagTasks, graphBoost, indexesBGL);
            paths.push_back(topoOrder);
        }

        // the order is generated by Boost Graph Library's topological sort
       std::vector<int> order = FindDependencyOrderDFS(dagTasks);
        paths.push_back(order);

        return paths;
    }
} // namespace OrderOptDAG_SPACE