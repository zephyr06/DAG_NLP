#include "SkipUnschedulablePermutations.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// std::vector<int> FindLongestChainJobIndex(const std::vector<RTDA> &rtdaVec)
// {
//     std::vector<int> index(rtdaVec.size(), 0);
//     for (uint i = 0; i < rtdaVec.size(); i++) // for each chain
//     {
//         std::vector<RTDA> &rtdaVec = rtdaVec[i];
//         auto ite = std::max_element(rtdaVec.begin(), rtdaVec.end(), [](RTDA r1, RTDA r2)
//                                     { return ObjRTDA(r1) < ObjRTDA(r2); });
//         index[i] = std::distance(rtdaVec.begin(), ite);
//     }
//     return index;
// }

// the time that the instance happens must be larger than the return value
double GetInstanceLeastStartTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo) {
  double prevInstanceLeastFinishTime = GetActivationTime(instance.job, tasksInfo);
  if (instance.type == 'f')
    prevInstanceLeastFinishTime += GetExecutionTime(instance.job, tasksInfo);
  return prevInstanceLeastFinishTime;
}

// the time that the instance happens must be smaller than the return value
double GetInstanceMaxFinishTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo) {
  double nextInstanceLeastFinishTime = GetDeadline(instance.job, tasksInfo);
  if (instance.type == 's')
    nextInstanceLeastFinishTime -= tasksInfo.tasks[instance.job.taskId].executionTime;
  return nextInstanceLeastFinishTime;
}
bool WhetherSkipInsertStart(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo,
                            const SFOrder &jobOrderCurr) {
  return WhetherSkipInsertStartByPreviousInstance(jobRelocate, startP, tasksInfo, jobOrderCurr) \
      || WhetherSkipInsertStartByFollowingInstance(jobRelocate, startP, tasksInfo, jobOrderCurr);
}
bool WhetherSkipInsertStartByPreviousInstance(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo,
                            const SFOrder &jobOrderCurr) {
  if (startP > 0) {
    TimeInstance instancePrev = jobOrderCurr.instanceOrder_[startP - 1];
    // jP.ActivationTime <= jR.start <= jR.deadline - jR.executionTime
    double prevInstanceLeastFinishTime = GetInstanceLeastStartTime(instancePrev, tasksInfo);
    if (prevInstanceLeastFinishTime >
        GetDeadline(jobRelocate, tasksInfo) - tasksInfo.tasks[jobRelocate.taskId].executionTime) {

      return true;
    }
  }
  return false;
}
bool WhetherSkipInsertStartByFollowingInstance(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo,
                            const SFOrder &jobOrderCurr) {
  if (startP < tasksInfo.length * 2 - 2) {
    TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[startP];
    //  jR.ActivationTime <= jR.start <= nextJ.Deadline
    double nextInstanceLeastFinishTime = GetInstanceMaxFinishTime(instanceAfter, tasksInfo);
    if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime) {

      return true;
    }
  }
  return false;
}

bool WhetherSkipInsertFinish(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo,
                             const SFOrder &jobOrderCurr) {
  return WhetherSkipInsertFinishByPreviousInstance(jobRelocate, finishP, tasksInfo, jobOrderCurr) \
      || WhetherSkipInsertFinishByFollowingInstance(jobRelocate, finishP, tasksInfo, jobOrderCurr);
}
bool WhetherSkipInsertFinishByPreviousInstance(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo,
                             const SFOrder &jobOrderCurr) {
  if (finishP > 0) {
    TimeInstance instancePrev = jobOrderCurr.instanceOrder_[finishP - 1];
    // jP.ActivationTime <= jR.finish <= jR.deadline
    double prevInstanceLeastFinishTime = GetInstanceLeastStartTime(instancePrev, tasksInfo);
    if (prevInstanceLeastFinishTime > GetDeadline(jobRelocate, tasksInfo)) {
      return true;
    }
  }
  return false;
}
bool WhetherSkipInsertFinishByFollowingInstance(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo,
                             const SFOrder &jobOrderCurr) {
  if (finishP < tasksInfo.length * 2 - 1) {
    TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[finishP];
    //  jR.ActivationTime <= jR.finish <= nextJ.Deadline
    double nextInstanceLeastFinishTime = GetInstanceMaxFinishTime(instanceAfter, tasksInfo);
    if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime) {
      return true;
    }
  }
  return false;
}

// TODO: WhetherStartFinishTooLong can probably be optimized for better efficiency
bool WhetherStartFinishTooLong(double &accumLengthMin, const JobCEC &jobRelocate, LLint finishP,
                               const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrderCurrForStart,
                               LLint startP) {
// #ifdef PROFILE_CODE
//   BeginTimer(__FUNCTION__);
// #endif

//   if (finishP >= 1 && finishP <= jobOrderCurrForStart.size()) {
//     TimeInstance jobPrevInsertInst = jobOrderCurrForStart.at(finishP - 1);
//     if (jobPrevInsertInst.type == 'f' &&
//         jobOrderCurrForStart.GetJobStartInstancePosition(jobPrevInsertInst.job) > startP) //
//       accumLengthMin += tasksInfo.tasks[jobPrevInsertInst.job.taskId].executionTime;
//   }
//   if (accumLengthMin >= tasksInfo.tasks[jobRelocate.taskId].executionTime) {
// #ifdef PROFILE_CODE
//     EndTimer(__FUNCTION__);
// #endif
//     return true;
//   }
// #ifdef PROFILE_CODE
//   EndTimer(__FUNCTION__);
// #endif
  return false;
}

bool SubGroupSchedulabilityCheck(JobGroupRange &jobGroup, SFOrder &jobOrderRef,
                                 const SFOrder &jobOrderCurrForFinish, LLint finishP, DAG_Model &dagTasks,
                                 const TaskSetInfoDerived &tasksInfo, int processorNum) {
  if (GlobalVariablesDAGOpt::enableSmallJobGroupCheck) {
    BeginTimer("FindUnschedulableSmallJobOrder");
    JobCEC jobNewlyAdded = jobOrderCurrForFinish.at(finishP - 1).job;
    jobGroup.minIndex = min(jobGroup.minIndex, jobOrderRef.GetJobStartInstancePosition(jobNewlyAdded));

    jobGroup.maxIndex = max(jobGroup.maxIndex, finishP);
    jobGroup.maxIndex = max(jobGroup.maxIndex, jobOrderRef.GetJobFinishInstancePosition(jobNewlyAdded) + 1);
    jobGroup.maxIndex = min(jobGroup.maxIndex, jobOrderRef.size());
    jobGroup.minIndex = max(jobGroup.minIndex, jobGroup.maxIndex - GlobalVariablesDAGOpt::subJobGroupMaxSize);
    jobGroup.minIndex = max(jobGroup.minIndex, 0);
    // countSubJobOrderLength.push_back(jobGroup.maxIndex - jobGroup.minIndex);
    std::vector<TimeInstance> instanceOrderSmall = ExtractSubInstances(jobOrderCurrForFinish, jobGroup);

    // BeginTimer("PrevSchedulabilityCheck");
    // bool bigFail = false;
    // if (SFOrderScheduling(dagTasks.tasks, tasksInfo, processorNum, jobOrderCurrForFinish)(0) == -1)
    // {
    //     bigFail = true;
    //     if (bigJobGroupCheck)
    //     {
    //         break;
    //     }
    // }
    // EndTimer("PrevSchedulabilityCheck");

    SFOrder jobOrderSmall(tasksInfo, instanceOrderSmall);

    bool smallFail = SFOrderScheduling(dagTasks.tasks, tasksInfo, processorNum, jobOrderSmall)(0) == -1;
    // if (bigFail == false && smallFail == true)
    // {
    //     if (GlobalVariablesDAGOpt::debugMode == 1)
    //         jobOrderSmall.print();
    //     // CoutWarning("One mismatched group check!");
    // }

    EndTimer("FindUnschedulableSmallJobOrder");
    if (smallFail)
      return true;
  }

  return false;
}
} // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
