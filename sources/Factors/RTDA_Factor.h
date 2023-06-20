#pragma once

#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Utils/JobCEC.h"

namespace OrderOptDAG_SPACE
{
  struct RTDA
  {
    double reactionTime;
    double dataAge;
    RTDA() : reactionTime(-1), dataAge(-1) {}
    RTDA(double r, double d) : reactionTime(r), dataAge(d) {}
    void print()
    {
      std::cout << "Reaction time is " << reactionTime << ", data age is " << dataAge << std::endl;
    }
  };

  RTDA GetMaxRTDA(const std::vector<RTDA> &resVec);

  std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo,
                                         const std::vector<int> &causeEffectChain, const VectorDynamic &x,
                                         double precision = GlobalVariablesDAGOpt::NumericalPrecision);

  std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo,
                                         const std::vector<int> &causeEffectChain, const gtsam::Values &x);

  RTDA GetMaxRTDA(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain,
                  const VectorDynamic &startTimeVector);

  double ObjRTDA(const RTDA &rtda);
  double ObjRTDA(const std::vector<RTDA> &rtdaVec);
  double ObjRT(const std::vector<RTDA> &rtdaVec);
  double ObjDA(const std::vector<RTDA> &rtdaVec);

  std::unordered_map<JobCEC, std::vector<JobCEC>>
  GetRTDAReactChainsFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain,
                                  const VectorDynamic &x);

  std::unordered_map<JobCEC, std::vector<JobCEC>>
  GetReactionChainMap(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                      int processorNum, const std::vector<int> &causeEffectChain, int chainIndex);

} // namespace OrderOptDAG_SPACE