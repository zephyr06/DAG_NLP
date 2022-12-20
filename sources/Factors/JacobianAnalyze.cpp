

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

std::unordered_map<JobCEC, size_t> GetJobOrderMap(const SFOrder &jobOrder) {
  int jobIndex = 0;
  std::unordered_map<JobCEC, size_t> jobIndexInJacobian;
  const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
  for (uint i = 0; i < instanceOrder.size(); i++) {
    if (instanceOrder[i].type == 's') {
      jobIndexInJacobian[instanceOrder[i].job] = jobIndex++;
    }
  }
  return jobIndexInJacobian;
}

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

std::vector<AugmentedJacobian> GetVariableBlocks(const DAG_Model &dagTasks,
                                                 const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder,
                                                 const std::vector<uint> processorJobVec, int processorNum) {
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

AugmentedJacobian
GetJacobianJobOrderReduced(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                           const SFOrder &jobOrder, int chainIndex,
                           const std::unordered_map<JobCEC, std::vector<JobCEC>> &reactionChainMap) {
  AugmentedJacobian augJacob;
  int maxRowNum = dagTasks.chains_[chainIndex].size() * reactionChainMap.size() * 2;
  augJacob.jacobian.conservativeResize(maxRowNum, tasksInfo.length);
  augJacob.jacobian.setZero();
  augJacob.rhs.conservativeResize(maxRowNum, 1);
  augJacob.rhs.setZero();
  int rowCount = 0;

  for (auto pair : reactionChainMap) {
    std::vector<JobCEC> &react_chain = pair.second;

    if (react_chain.size() > 1) {
      JobCEC pre_job_in_chain = react_chain[0];
      for (auto i = 1u; i < react_chain.size(); i++) {
        JobCEC cur_job = react_chain[i];

        // model_.add(GetFinishTimeExpression(pre_job_in_chain) <= GetStartTimeExpression(cur_job));
        int globalIdCurr = GetJobUniqueId(cur_job, tasksInfo);
        int globalIdPrev = GetJobUniqueId(pre_job_in_chain, tasksInfo);
        augJacob.jacobian(rowCount, globalIdPrev) = 1;
        augJacob.jacobian(rowCount, globalIdCurr) = -1;
        augJacob.rhs(rowCount) = -1 * GetExecutionTime(pre_job_in_chain, tasksInfo) +
                                 GetHyperPeriodDiff(pre_job_in_chain, cur_job, tasksInfo);
        rowCount++;

        JobCEC pre_cur_job(cur_job.taskId, cur_job.jobId - 1);
        if (pre_cur_job.jobId >= 0) {
          // Cplex only support weak inequality, a threshold is added to enforce strict inequality
          // model_.add(GetStartTimeExpression(pre_cur_job) <=
          //            GetFinishTimeExpression(pre_job_in_chain) -
          //                GlobalVariablesDAGOpt::kCplexInequalityThreshold);
          int globalIdPrevCurr = GetJobUniqueId(pre_cur_job, tasksInfo);
          augJacob.jacobian(rowCount, globalIdPrevCurr) = 1;
          augJacob.jacobian(rowCount, globalIdPrev) = -1;
          augJacob.rhs(rowCount) = GetExecutionTime(pre_job_in_chain, tasksInfo) -
                                   GlobalVariablesDAGOpt::kCplexInequalityThreshold +
                                   GetHyperPeriodDiff(pre_cur_job, pre_job_in_chain, tasksInfo);
          rowCount++;
        }
        pre_job_in_chain = cur_job;
      }
    }
  }
  augJacob.jacobian.conservativeResize(rowCount, tasksInfo.length);
  augJacob.rhs.conservativeResize(rowCount, 1);
  return augJacob;
}
} // namespace OrderOptDAG_SPACE