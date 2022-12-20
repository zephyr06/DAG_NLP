#include "sources/Factors/JacobianJobOrder.h"

namespace OrderOptDAG_SPACE {

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

void UpdateAugmentedJacobianJobOrder(AugmentedJacobian &augJacob, TimeInstance &instPrev,
                                     TimeInstance &instCurr, int rowIndex, int globalIdPrev, int globalIdCurr,
                                     const TaskSetInfoDerived &tasksInfo) {
  augJacob.jacobian.row(rowIndex).setZero();
  augJacob.jacobian(rowIndex, globalIdCurr) = -1;
  augJacob.jacobian(rowIndex, globalIdPrev) = 1;
  if (instPrev.type == 's') {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = 0;
    } else // instCurr.type == 'f'
    {
      augJacob.rhs(rowIndex) = GetExecutionTime(instCurr.job, tasksInfo);
    }
  } else // instPrev.type == 'f'
  {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
    } else // type == 'f'
    {
      augJacob.rhs(rowIndex) =
          -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
    }
  }
}

void UpdateAugmentedJacobianTripletJobOrder(AugmentedJacobianTriplet &augJacob, TimeInstance &instPrev,
                                            TimeInstance &instCurr, int rowIndex, int globalIdPrev,
                                            int globalIdCurr, const TaskSetInfoDerived &tasksInfo) {
  // augJacob.jacobian.row(rowIndex).setZero();
  // augJacob.jacobian(rowIndex, globalIdCurr) = -1;
  augJacob.jacobian.push_back(EigenTripletMy(rowIndex, globalIdCurr, -1));
  augJacob.jacobian.push_back(EigenTripletMy(rowIndex, globalIdPrev, 1));
  // augJacob.jacobian(rowIndex, globalIdPrev) = 1;
  if (instPrev.type == 's') {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = 0;
    } else // instCurr.type == 'f'
    {
      augJacob.rhs(rowIndex) = GetExecutionTime(instCurr.job, tasksInfo);
    }
  } else // instPrev.type == 'f'
  {
    if (instCurr.type == 's') {
      augJacob.rhs(rowIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
    } else // type == 'f'
    {
      augJacob.rhs(rowIndex) =
          -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
    }
  }
}

// Bug found: it can't correctly distinguish prev and next instances in cases such as (1,0,s) preceeds
// (0,0,f), but we'll probably not use this function anymore
AugmentedJacobianTriplet GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                             const SFOrder &jobOrder) {
  AugmentedJacobianTriplet augJacob;
  // augJacob.jacobian.conservativeResize(tasksInfo.length * 2 - 1, tasksInfo.length);
  // augJacob.jacobian.setZero();
  augJacob.rhs.conservativeResize(tasksInfo.length * 2 - 1, 1);
  augJacob.rhs.setZero();

  int count = 0;
  const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
  for (uint i = 1; i < instanceOrder.size(); i++) {
    auto instCurr = instanceOrder[i];
    auto instPrev = instanceOrder[i - 1];
    if (instPrev.job == instCurr.job)
      continue;
    int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo);
    int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo);
    UpdateAugmentedJacobianTripletJobOrder(augJacob, instPrev, instCurr, count, globalIdPrev, globalIdCurr,
                                           tasksInfo);
    count++;
  }

  // augJacob.jacobian.conservativeResize(count, tasksInfo.length);
  augJacob.rhs.conservativeResize(count, 1);
  augJacob.rows = count;
  augJacob.cols = tasksInfo.length;
  return augJacob;
}

AugmentedJacobianTriplet
GetJacobianJobOrderReduced(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                           const SFOrder &jobOrder, int chainIndex,
                           const std::unordered_map<JobCEC, std::vector<JobCEC>> &reactionChainMap) {
  AugmentedJacobianTriplet augJacob;
  int maxRowNum = dagTasks.chains_[chainIndex].size() * reactionChainMap.size() * 2;
  // augJacob.jacobian.conservativeResize(maxRowNum, tasksInfo.length);
  augJacob.jacobian.reserve(maxRowNum * 2);
  // augJacob.jacobian.setZero();
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
        // augJacob.jacobian(rowCount, globalIdPrev) = 1;
        augJacob.jacobian.push_back(EigenTripletMy(rowCount, globalIdPrev, 1));
        // augJacob.jacobian(rowCount, globalIdCurr) = -1;
        augJacob.jacobian.push_back(EigenTripletMy(rowCount, globalIdCurr, -1));
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
          // augJacob.jacobian(rowCount, globalIdPrevCurr) = 1;
          augJacob.jacobian.push_back(EigenTripletMy(rowCount, globalIdPrevCurr, 1));
          // augJacob.jacobian(rowCount, globalIdPrev) = -1;
          augJacob.jacobian.push_back(EigenTripletMy(rowCount, globalIdPrev, -1));

          augJacob.rhs(rowCount) = GetExecutionTime(pre_job_in_chain, tasksInfo) -
                                   GlobalVariablesDAGOpt::kCplexInequalityThreshold +
                                   GetHyperPeriodDiff(pre_cur_job, pre_job_in_chain, tasksInfo);
          rowCount++;
        }
        pre_job_in_chain = cur_job;
      }
    }
  }
  // augJacob.jacobian.conservativeResize(rowCount, tasksInfo.length);
  augJacob.rhs.conservativeResize(rowCount, 1);
  augJacob.rows = rowCount;
  augJacob.cols = tasksInfo.length;
  return augJacob;
}
} // namespace OrderOptDAG_SPACE