
#pragma once
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector, double tolerance = 1e-3);

// previous adjacent job means the jobs whose finish time equals the start time of jobCurr
// std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
//                                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                                            const VectorDynamic &startTimeVector);

// TODO: consider utilize startP and finishP
// bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
//                                   std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
//                                   const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                                   const VectorDynamic &startTimeVector,
//                                   std::unordered_set<JobCEC> &pathRecord, int &countPath);

// bool WhetherJobStartEarlier(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int>
// &jobGroupMap,
//                             SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                             const VectorDynamic &startTimeVector);

// 'backward' means finding the jobs whose index is larger than job, i.e., -->
// std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
//                                             const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                                             const VectorDynamic &startTimeVector);

// bool WhetherJobStartLaterHelper(JobCEC jobCurr, JobCEC jobChanged,
//                                 std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
//                                 const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                                 const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &pathRecord,
//                                 int &countPath);

// // this function requires that the start time is already maximum in startTimeVector
// // this function cannot work with SimpleOrderScheduler
// bool WhetherJobStartLater(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
//                           SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
//                           const VectorDynamic &startTimeVector);

} // namespace OrderOptDAG_SPACE