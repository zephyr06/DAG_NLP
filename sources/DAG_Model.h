#include "RegularTasks.h"

namespace DAG_SPACE
{
    typedef unordered_map<int, TaskSet> MAP_Prev;
    using namespace RegularTaskSystem;
    class DAG_Model
    {
    public:
        TaskSet tasks;
        MAP_Prev mapPrev;
        int N;

        DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev) : tasks(tasks),
                                                       mapPrev(mapPrev)
        {
            N = tasks.size();
        }

        void addEdge(int prevIndex, int nextIndex)
        {
            mapPrev[nextIndex].push_back(tasks[prevIndex]);
        }
    };

    DAG_Model GenerateRandomDAG(int N, double utilAll, double loadBalenceCoeff)
    {
        DAG_Model s;
        return s;
    }
}