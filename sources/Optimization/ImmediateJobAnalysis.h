
#pragma once
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE
{

    bool WhetherImmediateAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector, double tolerance = 1e-2);

    // returns whether instCompare is an immediate forward adjacent job to instCurr
    bool WhetherImmediateForwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                         const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                         const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                         double tolerance = 1e-2);
    std::vector<JobCEC> GetVectorFromSet(const std::unordered_set<JobCEC> &record);
    std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                               const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                               const VectorDynamic &startTimeVector);
    void FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                const VectorDynamic &startTimeVector, std::unordered_set<JobCEC> &record);
    // previous adjacent job means the jobs whose finish time equals the start time of jobCurr
    std::vector<JobCEC> FindForwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                               const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                               const VectorDynamic &startTimeVector);
    bool WhetherImmediateBackwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                          const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                          const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                          double tolerance = 1e-2);

    // 'backward' means finding the jobs whose index is larger than job, i.e., -->
    std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                                const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                                const VectorDynamic &startTimeVector);

} // namespace OrderOptDAG_SPACE