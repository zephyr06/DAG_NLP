#include "sources/Optimization/IterationStatus.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {

        int infeasibleCount = 0;
       
        template <typename OrderScheduler>
        bool MakeProgress(IterationStatus<OrderScheduler> &statusPrev, IterationStatus<OrderScheduler> &statusCurr)
        {
            if (!statusCurr.schedulable_)
            {
                infeasibleCount++;
                if (debugMode == 1)
                {
                    TaskSetInfoDerived tasksInfo(statusCurr.dagTasks_.tasks);
                    std::cout << "Infeasible schedule #:" << infeasibleCount << std::endl;
                    // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                    statusCurr.jobOrder_.print();
                }
                return false;
            }
            if (statusCurr.objWeighted_ < statusPrev.objWeighted_)
                return true;
            return false;
        }
    }
}