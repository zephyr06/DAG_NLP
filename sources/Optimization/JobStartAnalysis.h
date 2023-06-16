
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

  // TODO: consider utilize startP and finishP
  // Assumption: start time of jobCurr in startTimeVector cannot move earlier
  bool WhetherJobStartEarlierHelper(JobCEC jobCurr, JobCEC jobChanged,
                                    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                    const VectorDynamic &startTimeVector);

  bool WhetherJobStartEarlier(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                              SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                              const VectorDynamic &startTimeVector);

  bool WhetherImmediateBackwardAdjacent(const TimeInstance &instCurr, const TimeInstance &instCompare,
                                        const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                        const VectorDynamic &startTimeVector, SFOrder &jobOrder,
                                        double tolerance = 1e-2);

  // 'backward' means finding the jobs whose index is larger than job, i.e., -->
  std::vector<JobCEC> FindBackwardAdjacentJob(JobCEC job, SFOrder &jobOrder,
                                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                              const VectorDynamic &startTimeVector);

  bool WhetherJobStartLaterHelper(JobCEC jobCurr, JobCEC jobChanged,
                                  std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
                                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector);

  // this function requires that the start time is already maximum in startTimeVector
  // this function cannot work with SimpleOrderScheduler
  bool WhetherJobStartLater(JobCEC jobCurr, JobCEC jobChanged, std::unordered_map<JobCEC, int> &jobGroupMap,
                            SFOrder &jobOrder, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector);
  struct CentralJobs
  {
    CentralJobs(size_t size)
    {
      forwardJobs.reserve(size);
      backwardJobs.reserve(size);
    }
    CentralJobs(std::vector<JobCEC> &forwardJobs, std::vector<JobCEC> &backwardJobs)
        : forwardJobs(forwardJobs), backwardJobs(backwardJobs) {}

    std::vector<JobCEC> forwardJobs;
    std::vector<JobCEC> backwardJobs;
  };

  struct ActiveJob
  {
    ActiveJob() {}
    ActiveJob(JobCEC job, bool forward) : job(job), direction_forward(forward), direction_backward(!forward) {}
    JobCEC job;
    bool direction_forward;
    bool direction_backward;
  };

  struct ActiveJobs
  {
    ActiveJobs() {}
    ActiveJobs(const std::vector<ActiveJob> &activeJobs, const std::unordered_set<JobCEC> &jobRecord)
        : activeJobs(activeJobs), jobRecord(jobRecord)
    {
      for (uint i = 0; i < activeJobs.size(); i++)
      {
        auto &jobCurr = activeJobs[i];
        jobDirectionMap.insert({jobCurr.job, i});
      }
    }

    std::vector<JobCEC> GetJobs() const;
    inline size_t size() const { return activeJobs.size(); }
    inline ActiveJob operator[](size_t index) const { return activeJobs[index]; }
    inline bool FindDirection(JobCEC job) const
    {
      return activeJobs[jobDirectionMap.at(job)].direction_forward;
    }
    std::vector<ActiveJob> activeJobs;
    std::unordered_set<JobCEC> jobRecord;
    std::unordered_map<JobCEC, size_t> jobDirectionMap;
  };

  CentralJobs FindCentralJobs(const LongestCAChain &longestChain,
                              const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

  ActiveJobs FindActiveJobs(const CentralJobs &centralJobs, SFOrder &jobOrder,
                            const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector);

} // namespace OrderOptDAG_SPACE