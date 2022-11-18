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

            std::unordered_set<int> idSet;
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

        JobGroupRange FindJobActivateRange(const JobCEC &jobRelocate, SFOrder &jobOrderRef, const TaskSetInfoDerived &tasksInfo)
        {
            //  JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);
            LLint prevJobIndex = 0, nextJobIndex = static_cast<LLint>(jobOrderRef.size() - 1);
            if (jobRelocate.jobId > 0)
            {
                JobCEC prevJob(jobRelocate.taskId, jobRelocate.jobId - 1);
                prevJobIndex = jobOrderRef.GetJobFinishInstancePosition(prevJob);
            }
            if (jobRelocate.jobId < tasksInfo.sizeOfVariables[jobRelocate.taskId] - 1)
            {
                JobCEC nextJob(jobRelocate.taskId, jobRelocate.jobId + 1);
                nextJobIndex = std::min(jobOrderRef.GetJobStartInstancePosition(nextJob) + 1, nextJobIndex); // actually, I'm not sure why do we need this "+1", but it doesn't hurt to search for a few more
            }
            return JobGroupRange(prevJobIndex, nextJobIndex);
        }

    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
