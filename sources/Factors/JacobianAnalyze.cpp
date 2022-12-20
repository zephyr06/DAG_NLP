

#include "sources/Factors/JacobianAnalyze.h"

namespace OrderOptDAG_SPACE {

// GetDAGJacobianOrg
AugmentedJacobian GetDAGJacobianOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                    SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                    int processorNum, bool lessJobOrderConstraints) {
  AugmentedJacobian augJacoDDL = GetJacobianDDL(dagTasks, tasksInfo);
  AugmentedJacobian augJacoAct = GetJacobianActivationTime(dagTasks, tasksInfo);
  AugmentedJacobian augJacoDBF = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);

  AugmentedJacobian augJacoJobOrder;
  if (lessJobOrderConstraints) {
    std::vector<AugmentedJacobian> augJacobVec;
    augJacobVec.reserve(dagTasks.chains_.size());
    for (uint chainIndex = 0; chainIndex < dagTasks.chains_.size(); chainIndex++) {
      std::unordered_map<JobCEC, std::vector<JobCEC>> reactionChainMap =
          GetReactionChainMap(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum,
                              dagTasks.chains_[chainIndex], chainIndex);
      augJacobVec.push_back(
          GetJacobianJobOrderReduced(dagTasks, tasksInfo, jobOrder, chainIndex, reactionChainMap));
    }
    augJacoJobOrder = MergeAugJacobian(augJacobVec);
  } else
    augJacoJobOrder = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);

  AugmentedJacobian augJacobAll = StackAugJaco(augJacoDDL, augJacoAct);
  augJacobAll = StackAugJaco(augJacobAll, augJacoDBF);
  augJacobAll = StackAugJaco(augJacobAll, augJacoJobOrder);
  return augJacobAll;
}

// ****************************************Below is about ordered Jacobian *****************************

VectorDynamic ReOrderLPObj(const VectorDynamic &c, const SFOrder &jobOrder,
                           const TaskSetInfoDerived &tasksInfo) {
  std::unordered_map<JobCEC, size_t> jobIndexInJacobian = GetJobOrderMap(jobOrder);
  VectorDynamic cOrdered(tasksInfo.length);
  int count = 0;
  for (uint i = 0; i < tasksInfo.tasks.size(); i++) {
    for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++) {
      JobCEC jobCurr(i, j);
      cOrdered(jobIndexInJacobian[jobCurr]) = c(count++);
    }
  }
  return cOrdered;
}

std::vector<AugmentedJacobian>
GetVariableBlocksOrdered(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                         const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum) {
  int n = tasksInfo.length; // number of variables
  int m = 4;                // rows of Jacobian in each AugmentedJacobian

  // prepare the results initialization
  AugmentedJacobian jacobRef(m, n);
  // maximum rows: 2 for DDL and Acti, 2 for DBF, 2 for JobOrder
  jacobRef.jacobian.conservativeResize(1 + 1 + 2 + 2, n);
  jacobRef.rhs.conservativeResize(1 + 1 + 2 + 2, 1);
  std::vector<AugmentedJacobian> jacobs;
  jacobs.reserve(n);
  for (int i = 0; i < n; i++)
    jacobs.push_back(jacobRef);

  std::vector<int> rowCount(n);
  const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

  std::unordered_map<JobCEC, size_t> jobIndexInJacobian;
  int jobIndex = 0;
  for (uint i = 0; i < instanceOrder.size(); i++) {
    auto instCurr = instanceOrder[i];
    JobCEC jobCurr = instCurr.job;
    if (instCurr.type == 's') {
      UpdateAugmentedJacobianDDL(jacobs[jobIndex], 0, jobIndex, jobCurr, tasksInfo);

      UpdateAugmentedJacobianActivationTime(jacobs[jobIndex], 1, jobIndex, jobCurr, tasksInfo);

      jobIndexInJacobian[jobCurr] = jobIndex;
      rowCount[jobIndex] = 2;
      jobIndex++;
    }
  }

  // set DBF
  // TODO: clean code, probably create a struct for rowCount, jacobs, etc
  std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor =
      SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);

  for (uint processorId = 0; processorId < jobsOrderedEachProcessor.size(); processorId++) {
    const std::vector<JobCEC> &jobsOrdered = jobsOrderedEachProcessor[processorId];
    for (uint i = 1; i < jobsOrdered.size(); i++) {
      const JobCEC &jobPrev = jobsOrdered.at(i - 1);
      int globalIdPrev = jobIndexInJacobian[jobPrev];
      int globalIdCurr = jobIndexInJacobian[jobsOrdered.at(i)];
      // // let's assume that globalIdPrev happens earlier than globalIdCurr, which is actually guaranteed by
      // SortJobsEachProcessor(...)
      UpdateAugmentedJacobianDBF(jacobs[globalIdPrev], rowCount[globalIdPrev], globalIdPrev, globalIdCurr,
                                 jobPrev, tasksInfo);
      rowCount[globalIdPrev]++;
    }
  }

  // set job order
  for (uint i = 1; i < instanceOrder.size(); i++) {
    auto instCurr = instanceOrder[i];
    auto instPrev = instanceOrder[i - 1];

    if (instPrev.job == instCurr.job)
      continue; // setting jacobian and rhs as 0 vectors

    int globalIdPrev = jobIndexInJacobian[instPrev.job];
    int globalIdCurr = jobIndexInJacobian[instCurr.job];

    int jacobianIndexToUpdate = std::min(globalIdPrev, globalIdCurr);
    UpdateAugmentedJacobianJobOrder(jacobs[jacobianIndexToUpdate], instPrev, instCurr,
                                    rowCount[jacobianIndexToUpdate], globalIdPrev, globalIdCurr, tasksInfo);
    rowCount[jacobianIndexToUpdate]++;
  }

  jobIndex = 0;
  for (auto &augJacob : jacobs) {
    augJacob.jacobian.conservativeResize(rowCount[jobIndex], n);
    augJacob.rhs.conservativeResize(rowCount[jobIndex++], 1);
  }

  return jacobs;
}

} // namespace OrderOptDAG_SPACE