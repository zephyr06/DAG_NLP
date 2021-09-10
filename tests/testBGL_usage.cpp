#include <boost/config.hpp>
#include <iostream>          // for std::cout
#include <utility>           // for std::pair
#include <algorithm>         // for std::for_each
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

// struct DependencyTask
// {
//     string name;
// };
struct first_name_t
{
    typedef vertex_property_tag kind;
};

template <class EdgeIter, class Graph>
void who_owes_who(EdgeIter first, EdgeIter last, const Graph &G)
{
    // Access the propety acessor type for this graph
    typedef typename property_map<Graph,
                                  first_name_t>::const_type NameMap;
    NameMap name = get(first_name_t(), G);

    typedef typename boost::property_traits<NameMap>::value_type NameType;

    NameType src_name, targ_name;

    while (first != last)
    {
        src_name = boost::get(name, source(*first, G));
        targ_name = boost::get(name, target(*first, G));
        // cout << src_name << " owes "
        //      << targ_name << " some money" << endl;
        cout << src_name.id << ", " << targ_name.id << endl;
        ++first;
    }
}

int main()
{
    using namespace RegularTaskSystem;
    using namespace DAG_SPACE;
    cout << "Hello" << endl;
    DAG_Model dagTasks = ReadDAG_Tasks("../TaskData/test_n5_v15.csv", "orig");
    int N = dagTasks.tasks.size();

    typedef property<first_name_t, Task> FirstNameProperty;
    typedef adjacency_list<vecS, vecS, bidirectionalS, FirstNameProperty> Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_t;
    typedef boost::graph_traits<Graph>::edge_descriptor edge_t;

    // typedef std::pair<Task, Task> Edge;
    Graph g(dagTasks.tasks.size());
    // for (int i = 0; i < N; i++)
    // {
    //     vertex_t u = boost::add_vertex(g);
    //     g[u] = dagTasks.tasks[i];
    //     // edge_t v =
    // }

    for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
    {
        const TaskSet &tasksPrev = itr->second;
        size_t indexNext = itr->first;
        for (size_t i = 0; i < tasksPrev.size(); i++)
        {
            boost::add_edge(tasksPrev[i].id, dagTasks.tasks[indexNext].id, g);
        }
    }
    property_map<Graph, first_name_t>::type
        name = get(first_name_t(), g);
    for (int i = 0; i < 5; i++)
        boost::put(name, i, dagTasks.tasks[i]);
    who_owes_who(edges(g).first, edges(g).second, g);

    typedef std::list<vertex_t> MakeOrder;
    MakeOrder make_order;
    boost::topological_sort(g, std::front_inserter(make_order));

    std::cout << "dependency ordering: ";
    for (MakeOrder::iterator i = make_order.begin();
         i != make_order.end(); ++i)
    // std::cout << (*i).print() << " ";
    {
        cout << *i << endl;
        // cout << i << endl;
    }
    // i->print();
    std::cout << std::endl;
    // boost::write_graphviz(
    //     std::cout, g, [&](auto &out, auto v)
    //     { out << "[label=\"" << g[v].executionTime << "\"]"; },
    //     [&](auto &out, auto e)
    //     {
    //         out << "[label=\""
    //             << "\"]";
    //     });
    // std::cout << std::flush;

    // write_graphviz(cout, g);
    return 0;
}