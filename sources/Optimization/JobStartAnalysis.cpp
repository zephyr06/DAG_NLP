
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector, double tolerance) {
  if (instCurr.type == 's') {
    if (instCompare.type == 's') {
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetStartTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    }
  } else { // instCurr.type=='f'
    if (instCompare.type == 's') {
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetStartTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    } else { // instCompare.type=='f'
      if (std::abs(GetFinishTime(instCurr.job, startTimeVector, tasksInfo) -
                   GetFinishTime(instCompare.job, startTimeVector, tasksInfo)) < tolerance) {
        return true;
      }
    }
  }

  return false;
}

} // namespace OrderOptDAG_SPACE