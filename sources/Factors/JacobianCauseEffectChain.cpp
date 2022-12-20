#include "sources/Factors/JacobianCauseEffectChain.h"

namespace OrderOptDAG_SPACE {

// Each one or two rows correspond to one chain, first RT constraint, then DA constraint;
// if DA data is overwritten, then we'll skip this row;
// As for the added variable, it's also RT first DA second;
// jobOrder's jobSFMap_ may be updated
// TODO: this function only considres 1 chain, add functiosn for multiple chains
AugmentedJacobianTriplet
GetJacobianCauseEffectChainOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                               SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum,
                               const std::vector<int> &causeEffectChain, int chainIndex) {
  std::unordered_map<JobCEC, JobCEC> firstReactionMap;

  LLint hyperPeriod = tasksInfo.hyperPeriod;
  const TaskSet &tasks = tasksInfo.tasks;
  LLint totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;

  AugmentedJacobianTriplet augJacob;
  int varNum = tasksInfo.length + 2 * (1 + chainIndex);
  // augJacob.jacobian.conservativeResize((totalStartJobs + 1) * 2, varNum);
  augJacob.jacobian.reserve((totalStartJobs + 1) * 2 * 3);
  augJacob.rhs.conservativeResize((totalStartJobs + 1) * 2, 1);
  int varIndexRT = tasksInfo.length + 2 * (chainIndex);
  int varIndexDA = tasksInfo.length + 1 + 2 * (chainIndex);

  if (causeEffectChain.size() == 0) {
    return augJacob;
  }

  int rowIndex = 0;
  for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++) {
    JobCEC firstJob = {causeEffectChain[0], (startInstanceIndex)};
    std::vector<JobCEC> react_chain;
    react_chain.push_back(firstJob);
    for (uint j = 1; j < causeEffectChain.size(); j++) {
      LLint instIndexFirstJob = jobOrder.GetJobFinishInstancePosition(firstJob);
      LLint jobIndex = 0;
      while (true) {
        JobCEC jobCurr{causeEffectChain[j], jobIndex};
        if (jobOrder.GetJobStartInstancePosition(jobCurr) > instIndexFirstJob)
          break;
        jobIndex++;
        if (jobIndex > 10000) {
          CoutError("didn't find a match in GetJacobianCauseEffectChainOrg!");
        }
      }
      firstJob = {causeEffectChain[j], jobIndex};
      react_chain.push_back(firstJob);
    }

    JobCEC startJobCurr(causeEffectChain[0], startInstanceIndex);
    firstReactionMap[startJobCurr] = firstJob;

    // update augJacob
    // augJacob.jacobian.row(rowIndex).setZero();
    int sourceJobCol = GetJobUniqueId(startJobCurr, tasksInfo);
    int tailJobCol = GetJobUniqueId(firstJob, tasksInfo);
    // augJacob.jacobian(rowIndex, tailJobCol) = 1;
    augJacob.jacobian.push_back(EigenTripletMy(rowIndex, tailJobCol, 1));
    // augJacob.jacobian(rowIndex, sourceJobCol) = -1;
    augJacob.jacobian.push_back(EigenTripletMy(rowIndex, sourceJobCol, -1));
    // augJacob.jacobian(rowIndex, varIndexRT) = -1;
    augJacob.jacobian.push_back(EigenTripletMy(rowIndex, varIndexRT, -1));

    augJacob.rhs(rowIndex) =
        -1 * GetExecutionTime(firstJob, tasksInfo) - GetHyperPeriodDiff(startJobCurr, firstJob, tasksInfo);
    rowIndex++;

    // update data age
    JobCEC firstReactLastJob = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
    if (startInstanceIndex > 0 && firstReactionMap[firstReactLastJob] != firstJob && firstJob.jobId > 0) {
      JobCEC lastReaction = firstJob;
      lastReaction.jobId--;
      // resVec[startInstanceIndex - 1].dataAge = GetStartTime(lastReaction, x, tasksInfo) +
      // tasks[lastReaction.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex -
      // 1}, x, tasksInfo);
      // augJacob.jacobian.row(rowIndex).setZero();
      JobCEC sourceJobDA = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
      int sourceJobColDA = GetJobUniqueIdWithinHyperPeriod(sourceJobDA, tasksInfo);
      int tailJobColDA = GetJobUniqueIdWithinHyperPeriod(lastReaction, tasksInfo);
      // augJacob.jacobian(rowIndex, tailJobColDA) = 1;
      augJacob.jacobian.push_back(EigenTripletMy(rowIndex, tailJobColDA, 1));
      // augJacob.jacobian(rowIndex, sourceJobColDA) = -1;
      augJacob.jacobian.push_back(EigenTripletMy(rowIndex, sourceJobColDA, -1));
      // augJacob.jacobian(rowIndex, varIndexDA) = -1;
      augJacob.jacobian.push_back(EigenTripletMy(rowIndex, varIndexDA, -1));
      augJacob.rhs(rowIndex) = -1 * GetExecutionTime(lastReaction, tasksInfo) -
                               GetHyperPeriodDiff(sourceJobDA, lastReaction, tasksInfo);
      rowIndex++;
    }
  }
  // augJacob.jacobian.conservativeResize(rowIndex, varNum);
  augJacob.rhs.conservativeResize(rowIndex, 1);

  augJacob.rows = rowIndex;
  augJacob.cols = varNum;
  return augJacob;
}

AugmentedJacobian MergeJacobianOfRTDAChains(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                            SFOrder &jobOrder, const std::vector<uint> processorJobVec,
                                            int processorNum) {
  AugmentedJacobian jacobAll;
  int mAllMax = 0, nAll = tasksInfo.length + 2 * dagTasks.chains_.size();
  for (uint i = 0; i < dagTasks.chains_.size(); i++) {
    mAllMax += 2 * (tasksInfo.hyperPeriod / dagTasks.tasks[dagTasks.chains_[i][0]].period + 1 +
                    1); // +1 just to be safe
  }
  jacobAll.jacobian.conservativeResize(mAllMax, nAll);
  // TODO: verify whether this will introduce extra 0s
  jacobAll.jacobian.setZero();
  jacobAll.rhs.conservativeResize(mAllMax, 1);

  // TODO: improve efficiency in matrix merge;
  int rowCount = 0;
  int colCount = -1;
  for (uint i = 0; i < dagTasks.chains_.size(); i++) {
    AugmentedJacobian augJacobRTDACurr = GetJacobianCauseEffectChainOrg(
        dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum, dagTasks.chains_[i], i);
    jacobAll.jacobian.block(rowCount, 0, augJacobRTDACurr.jacobian.rows(), augJacobRTDACurr.jacobian.cols()) =
        augJacobRTDACurr.jacobian;
    jacobAll.rhs.block(rowCount, 0, augJacobRTDACurr.jacobian.rows(), 1) = augJacobRTDACurr.rhs;
    rowCount += augJacobRTDACurr.jacobian.rows();
    colCount = augJacobRTDACurr.jacobian.cols();
  }
  jacobAll.jacobian.conservativeResize(rowCount, colCount);
  jacobAll.rhs.conservativeResize(rowCount);
  return jacobAll;
}
} // namespace OrderOptDAG_SPACE