#include "sources/Factors/JacobianDBF.h"

namespace OrderOptDAG_SPACE {

// For each processor, the jobs are ordered following the gien jobOrder
std::vector<std::vector<JobCEC>>
SortJobsEachProcessor(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder,
                      const std::vector<uint> processorJobVec, int processorNum) {
  const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

  std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor(processorNum);
  // jobsOrderedEachProcessor.reserve(processorNum);
  for (uint i = 0; i < instanceOrder.size(); i++) {
    TimeInstance instCurr = instanceOrder[i];
    if (instCurr.type == 's') // only exam start instances
    {
      JobCEC &jobCurr = instCurr.job;
      LLint uniqueJobId = GetJobUniqueId(instCurr.job, tasksInfo);
      jobsOrderedEachProcessor[processorJobVec[uniqueJobId]].push_back(jobCurr);
    }
  }

  // remove idle processor
  int i = jobsOrderedEachProcessor.size() - 1;
  for (; i >= 0; i--) {
    if (jobsOrderedEachProcessor[i].empty())
      jobsOrderedEachProcessor.erase(jobsOrderedEachProcessor.begin() + i);
  }

  return jobsOrderedEachProcessor;
}

AugmentedJacobianTriplet GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                        const SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                        int processorNum) {
  // find all the jobs that are executed on the same processor

  // For each processor, sort all the jobs' execution order

  std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor =
      SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
  // Add DBF constraints for each adjacent job pair

  int m = tasksInfo.length - 1 - (static_cast<int>(jobsOrderedEachProcessor.size()) - 1);

  AugmentedJacobianTriplet augJacob;
  // augJacob.jacobian = GenerateMatrixDynamic(m, tasksInfo.length);
  augJacob.jacobian.reserve(2 * m);
  augJacob.rhs = GenerateVectorDynamic(m);
  int count = 0;
  for (uint processorId = 0; processorId < jobsOrderedEachProcessor.size(); processorId++) {
    const std::vector<JobCEC> &jobsOrdered = jobsOrderedEachProcessor[processorId];
    for (uint i = 1; i < jobsOrdered.size(); i++) {
      int globalIdPrev = GetJobUniqueId(jobsOrdered.at(i - 1), tasksInfo);
      int globalIdCurr = GetJobUniqueId(jobsOrdered.at(i), tasksInfo);
      // UpdateAugmentedJacobianDBF(augJacob, count, globalIdPrev, globalIdCurr, jobsOrdered.at(i - 1),
      //                            tasksInfo);
      UpdateAugmentedJacobianTripletDBF(augJacob, count, globalIdPrev, globalIdCurr, jobsOrdered.at(i - 1),
                                        tasksInfo);
      count++;
    }
  }

  return augJacob;
}

} // namespace OrderOptDAG_SPACE