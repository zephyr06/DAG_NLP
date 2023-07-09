
#pragma once
#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Factors/WorstSF_Fork.h"
#include "sources/Optimization/ImmediateJobAnalysis.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

// TODO: consider utilize startP and finishP
// Assumption: start time of jobCurr in startTimeVector cannot move earlier
bool WhetherJobStartEarlierHelper(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);

bool WhetherJobStartEarlier(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);

bool WhetherJobStartLaterHelper(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);

// this function requires that the start time is already maximum in
// startTimeVector this function cannot work with SimpleOrderScheduler
bool WhetherJobStartLater(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);

struct CentralJobs {
    CentralJobs() {}
    CentralJobs(size_t size) {
        forwardJobs.reserve(size);
        backwardJobs.reserve(size);
    }
    CentralJobs(std::vector<JobCEC> &forwardJobs,
                std::vector<JobCEC> &backwardJobs)
        : forwardJobs(forwardJobs), backwardJobs(backwardJobs) {}

    // TODO: make the name consistent with standard definitions in cause-effect
    // chains
    std::vector<JobCEC> forwardJobs;  // e.g., sink jobs in cause-effect chains
    std::vector<JobCEC>
        backwardJobs;  // e.g., source jobs in cause-effect chains
};

struct ActiveJob {
    ActiveJob() {}
    ActiveJob(JobCEC job, bool forward)
        : job(job), direction_forward(forward), direction_backward(!forward) {}
    JobCEC job;
    bool direction_forward;
    bool direction_backward;
};

struct ActiveJobs {
    ActiveJobs() {}
    ActiveJobs(const std::vector<ActiveJob> &activeJobs,
               const std::unordered_set<JobCEC> &jobRecord)
        : activeJobs(activeJobs), jobRecord(jobRecord) {
        for (uint i = 0; i < activeJobs.size(); i++) {
            auto &jobCurr = activeJobs[i];
            jobDirectionMap.insert({jobCurr.job, i});
        }
    }

    std::vector<JobCEC> GetJobs() const;
    inline size_t size() const { return activeJobs.size(); }
    inline ActiveJob operator[](size_t index) const {
        return activeJobs[index];
    }
    inline bool FindDirection(JobCEC job) const {
        return activeJobs[jobDirectionMap.at(job)].direction_forward;
    }

    // data members
    std::vector<ActiveJob> activeJobs;
    std::unordered_set<JobCEC> jobRecord;
    std::unordered_map<JobCEC, size_t> jobDirectionMap;
};

CentralJobs FindCentralJobs(
    const LongestCAChain &longestChain,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

CentralJobs FindCentralJobs(
    const WorstSF_JobFork &worst_sf_fork, const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

ActiveJobs FindActiveJobs(
    const CentralJobs &centralJobs, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);

// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove the job, and then insert its
// start/finish instances at startP/finishP
// If you want to be safer, return more 'true'
bool WhetherJobBreakChainRT(const JobCEC &job, LLint startP, LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo);
bool WhetherJobBreakChainDA(const JobCEC &job, LLint startP, LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo);
bool WhetherJobBreakChainSF(const JobCEC &job, LLint startP, LLint finishP,
                            const WorstSF_JobFork &worst_sf_fork,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo);

bool WhetherJobBreakChainRTDA(const JobCEC &job, LLint startP, LLint finishP,
                              const LongestCAChain &longestJobChains,
                              const DAG_Model &dagTasks, SFOrder &jobOrder,
                              const TaskSetInfoDerived &tasksInfo,
                              std::string obj_type);

class IndependentAnalysis {
   public:
    IndependentAnalysis() {}

    IndependentAnalysis(const DAG_Model &dagInput,
                        const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                        const VectorDynamic &startTimeVector, int processorNum,
                        std::string obj_type)
        : obj_type_(obj_type),
          dagTasks_(dagInput),
          tasksInfo_(tasksInfo),
          processorNum_(processorNum) {
        UpdateStatus(jobOrder, startTimeVector);
    }

    inline bool WhetherInfluenceActiveJobs(JobCEC jobRelocate) {
        return activeJobs_.jobRecord.find(jobRelocate) !=
               activeJobs_.jobRecord.end();
    }

    void UpdateStatus(SFOrder &jobOrder, const VectorDynamic &startTimeVector) {
        CentralJobs centralJob;
        if (obj_type_ == "ReactionTimeObj" || obj_type_ == "DataAgeObj") {
            longestJobChains_ =
                LongestCAChain(dagTasks_, tasksInfo_, jobOrder, startTimeVector,
                               processorNum_, obj_type_);
            // TODO: add this function or not?
            // jobGroupMap_ = ExtractIndependentJobGroups(jobOrderRef,
            // tasksInfo);
            centralJob = FindCentralJobs(longestJobChains_, tasksInfo_);
        } else if (obj_type_ == "SensorFusionObj") {
            worst_sf_fork_ = WorstSF_JobFork(dagTasks_, tasksInfo_, jobOrder,
                                             startTimeVector, processorNum_);
            centralJob =
                FindCentralJobs(worst_sf_fork_, startTimeVector, tasksInfo_);
        } else
            CoutError("Need implementation for obj_type_ in UpdateStatus");
        activeJobs_ =
            FindActiveJobs(centralJob, jobOrder, tasksInfo_, startTimeVector);
    }

    bool WhetherSafeSkip(const JobCEC jobRelocate, LLint startP, LLint finishP,
                         SFOrder &jobOrderRef) {
        if (obj_type_ == "ReactionTimeObj" || obj_type_ == "DataAgeObj") {
            if (!WhetherInfluenceActiveJobs(jobRelocate) &&
                !WhetherJobBreakChainRTDA(jobRelocate, startP, finishP,
                                          longestJobChains_, dagTasks_,
                                          jobOrderRef, tasksInfo_, obj_type_)) {
                return true;
            } else
                return false;
        } else if (obj_type_ == "SensorFusionObj") {
            if (!WhetherInfluenceActiveJobs(jobRelocate) &&
                !WhetherJobBreakChainSF(jobRelocate, startP, finishP,
                                        worst_sf_fork_, dagTasks_, jobOrderRef,
                                        tasksInfo_)) {
                return true;
            } else
                return false;
        }
        CoutError("Need implementation in WhetherSafeSkip");
        return false;
    }

    // data members
    std::string obj_type_;
    DAG_Model dagTasks_;
    TaskSetInfoDerived tasksInfo_;
    int processorNum_;

    ActiveJobs activeJobs_;

    // for RT / DA
    LongestCAChain longestJobChains_;

    // for SF
    WorstSF_JobFork worst_sf_fork_;

    // not used for now
    std::unordered_map<JobCEC, int> jobGroupMap_;
};

}  // namespace OrderOptDAG_SPACE