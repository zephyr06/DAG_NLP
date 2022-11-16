#include "OptimizeSFOrder.h"

namespace OrderOptDAG_SPACE
{
    // template <typename OrderScheduler>
    namespace OptimizeSF
    {

        std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks)
        {
            std::vector<int> idVec;
            idVec.reserve(dagTasks.tasks.size());
            if (dagTasks.chains_.size() == 0)
            {
                return idVec;
            }
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
                std::copy(dagTasks.chains_[i].begin(), dagTasks.chains_[i].end(), back_inserter(idVec));
            if (enableFastSearch)
                return idVec;

            unordered_set<int> idSet;
            std::vector<int> idVecChainFirst = dagTasks.chains_[0];
            for (uint i = 0; i < dagTasks.chains_[0].size(); i++)
                idSet.insert(dagTasks.chains_[0][i]);

            for (uint i = 0; i < dagTasks.tasks.size(); i++)
            {
                if (idSet.find(i) == idSet.end())
                    idVec.push_back(i);
            }
            return idVec;
        }

    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
