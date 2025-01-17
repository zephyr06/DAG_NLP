#include "sources/Optimization/OptimizeSFOrder_TOM.h"

namespace OrderOptDAG_SPACE {
// template <typename OrderScheduler>
namespace OptimizeSF {
std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks) {
    std::vector<int> idVec;
    std::unordered_set<int> idSet;
    idVec.reserve(dagTasks.tasks.size());
    if (dagTasks.chains_.size() == 0) {
        return idVec;
    }
    for (uint i = 0; i < dagTasks.chains_.size(); i++)
        for (uint j = 0; j < dagTasks.chains_[i].size(); j++)
            if (idSet.find(dagTasks.chains_[i][j]) == idSet.end()) {
                idSet.insert(dagTasks.chains_[i][j]);
                idVec.push_back(dagTasks.chains_[i][j]);
            }
    if (GlobalVariablesDAGOpt::enableFastSearch)
        return idVec;

    for (uint i = 0; i < dagTasks.tasks.size(); i++) {
        if (idSet.find(i) == idSet.end())
            idVec.push_back(i);
    }
    return idVec;
}

JobGroupRange FindJobActivateRange(const JobCEC &jobRelocate,
                                   SFOrder &jobOrderRef,
                                   const TaskSetInfoDerived &tasksInfo) {
    LLint prevJobIndex = 0, nextJobIndex = 1e5;
    if (jobRelocate.jobId > 0) {
        JobCEC prevJob(jobRelocate.taskId, jobRelocate.jobId - 1);
        prevJobIndex = jobOrderRef.GetJobFinishInstancePosition(prevJob);
    }
    if (jobRelocate.jobId < tasksInfo.sizeOfVariables[jobRelocate.taskId] - 1) {
        JobCEC nextJob(jobRelocate.taskId, jobRelocate.jobId + 1);
        nextJobIndex = std::min(
            jobOrderRef.GetJobStartInstancePosition(nextJob), nextJobIndex);
    }
    JobGroupRange range(prevJobIndex, nextJobIndex);
    // go through other tasks to further bound the interval
    for (uint i = 0; i < tasksInfo.tasks.size(); i++) {
        if (i == jobRelocate.taskId)
            continue;
        int startTimeJobRelocate = GetActivationTime(jobRelocate, tasksInfo);
        int prevInstance = startTimeJobRelocate / tasksInfo.tasks[i].period - 1;
        if (prevInstance >= 0) {
            JobCEC jobPrev(i, prevInstance);
            if (GetDeadline(jobPrev, tasksInfo) > startTimeJobRelocate)
                jobPrev.jobId--;
            range.minIndex = std::max(
                range.minIndex,
                int(jobOrderRef.GetJobFinishInstancePosition(jobPrev)));
        }

        int nextInstance = ceil(GetDeadline(jobRelocate, tasksInfo) /
                                double(tasksInfo.tasks[i].period));
        if (nextInstance < tasksInfo.hyper_period / tasksInfo.tasks[i].period) {
            JobCEC jobNext(i, nextInstance);
            range.maxIndex =
                std::min(range.maxIndex,
                         int(jobOrderRef.GetJobStartInstancePosition(jobNext)));
        }
    }

    return range;
}

void CheckValidDAGTaskSetGivenObjType(const DAG_Model &dagTasks,
                                      std::string obj_type) {
    if (obj_type == "ReactionTimeObj" || obj_type == "DataAgeObj") {
        if (dagTasks.chains_.size() == 0)
            CoutError(
                "Need to provide cause-effect chains to perform optimization!");
    } else if (obj_type == "SensorFusionObj") {
        if (dagTasks.sf_forks_.size() == 0) {
            CoutError(
                "Need to provide sensor fusion forks to perform optimization!");
        }
    }
}
}  // namespace OptimizeSF
}  // namespace OrderOptDAG_SPACE
