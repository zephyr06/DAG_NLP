
#include "sources/Optimization/IndependentAnalysis.h"

#include "sources/Factors/LongestChain.h"
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {
// TODO: consider utilize startP and finishP
// Assumption: start time of jobCurr in startTimeVector cannot move earlier
bool WhetherJobStartEarlierHelper(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    // ,
    // std::unordered_set<JobCEC> &pathRecord, int &countPath
    // if (countPath > 10)
    //   return true;
    if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) -
                 GetActivationTime(jobCurr, tasksInfo)) < 1e-3)
        return false;
    else if (jobCurr == jobChanged) {
        // countPath++;
        return true;
    }

    std::vector<JobCEC> prevAdjacentJobs =
        FindForwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    if (prevAdjacentJobs.size() == 0)
        return false;  // consider more, should be false under assumptions  of
                       // startTimeVector

    for (auto &jobPrev : prevAdjacentJobs) {
        if (WhetherJobStartEarlierHelper(jobPrev, jobChanged, jobGroupMap,
                                         jobOrder, tasksInfo,
                                         startTimeVector) == false)
            return false;
    }

    return true;  // all the conditions known to return false failed
}

bool WhetherJobStartEarlier(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

    // Current analysis on job group is not safe, let's' see what we can do
    // without it if (!WhetherInfluenceJobSimple(jobCurr, jobChanged,
    // jobGroupMap))
    //   return false;
    return WhetherJobStartEarlierHelper(jobCurr, jobChanged, jobGroupMap,
                                        jobOrder, tasksInfo, startTimeVector);
}

bool WhetherJobStartLaterHelper(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    if (std::abs(GetFinishTime(jobCurr, startTimeVector, tasksInfo) -
                 GetDeadline(jobCurr, tasksInfo)) < 1e-3)
        return false;
    else if (jobCurr == jobChanged) {
        return true;
    }

    std::vector<JobCEC> followAdjacentJobs =
        FindBackwardAdjacentJob(jobCurr, jobOrder, tasksInfo, startTimeVector);
    if (followAdjacentJobs.size() ==
        0)  // this should never happen in case of LP order scheduler, and this
            // function should not be used with simple order scheduler
        return false;

    for (auto &jobFollow : followAdjacentJobs) {
        if (WhetherJobStartLaterHelper(jobFollow, jobChanged, jobGroupMap,
                                       jobOrder, tasksInfo,
                                       startTimeVector) == false)
            return false;
    }

    return true;
}

// this function requires that the start time is already maximum in
// startTimeVector this function cannot work with SimpleOrderScheduler
bool WhetherJobStartLater(
    JobCEC jobCurr, JobCEC jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    jobChanged = jobChanged.GetJobWithinHyperPeriod(tasksInfo);

    // Current analysis on job group is not safe, let's' see what we can do
    // without it if (!WhetherInfluenceJobSimple(jobCurr, jobChanged,
    // jobGroupMap))
    //   return false;
    // int countPath = 0;
    // std::unordered_set<JobCEC> pathRecord;
    // pathRecord.reserve(tasksInfo.length);

    return WhetherJobStartLaterHelper(jobCurr, jobChanged, jobGroupMap,
                                      jobOrder, tasksInfo, startTimeVector);
}

std::vector<JobCEC> ActiveJobs::GetJobs() const {
    std::vector<JobCEC> jobs;
    jobs.reserve(size());
    for (auto &jobA : activeJobs) jobs.push_back(jobA.job);
    return jobs;
}

CentralJobs FindCentralJobs(
    const LongestCAChain &longestChain,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
    std::unordered_set<JobCEC> centralSourceJobRecord;
    centralSourceJobRecord.reserve(tasksInfo.length);
    std::unordered_set<JobCEC> centralSinkJobRecord;
    centralSinkJobRecord.reserve(tasksInfo.length);
    CentralJobs centralJobs(tasksInfo.length);

    for (uint i = 0; i < longestChain.size(); i++) {
        auto &chainCurr = longestChain.longestChains_[i];
        if (chainCurr.size() > 0) {
            JobCEC jobSource = chainCurr[0].GetJobWithinHyperPeriod(tasksInfo);
            if (centralSourceJobRecord.find(jobSource) ==
                centralSourceJobRecord.end()) {
                centralSourceJobRecord.insert(jobSource);
                centralJobs.backwardJobs.push_back(jobSource);
            }

            JobCEC jobSink =
                ((chainCurr.back())).GetJobWithinHyperPeriod(tasksInfo);
            if (centralSinkJobRecord.find(jobSink) ==
                centralSinkJobRecord.end()) {
                centralSinkJobRecord.insert(jobSink);
                centralJobs.forwardJobs.push_back(jobSink);
            }
        }
    }
    return centralJobs;
}

CentralJobs FindCentralJobs(
    const WorstSF_JobFork &worst_sf_fork, const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
    std::unordered_set<JobCEC> centralSourceJobRecord;
    centralSourceJobRecord.reserve(tasksInfo.length);
    std::unordered_set<JobCEC> centralSinkJobRecord;
    centralSinkJobRecord.reserve(tasksInfo.length);
    CentralJobs centralJobs(worst_sf_fork.size() *
                            5);  // safe but not necessary reservation
    for (const auto &sf_fork : worst_sf_fork.worst_fork_) {
        // for (JobCEC job : sf_fork.source_jobs) {
        //     job = job.GetJobWithinHyperPeriod(tasksInfo);
        //     if (centralSourceJobRecord.count(job) == 0) {
        //         centralSourceJobRecord.insert(job);
        //         centralJobs.backwardJobs.push_back(job);
        //     }
        // }
        EarliestAndLatestFinishedJob early_late_jobs =
            FindEarlyAndLateJob(sf_fork, startTimeVector, tasksInfo);

        // add the earliest job
        JobCEC early_job =
            early_late_jobs.early_job.GetJobWithinHyperPeriod(tasksInfo);
        if (centralSourceJobRecord.count(early_job) == 0) {
            centralSourceJobRecord.insert(early_job);
            centralJobs.backwardJobs.push_back(early_job);
        }
        // add the latest job
        JobCEC late_job =
            early_late_jobs.late_job.GetJobWithinHyperPeriod(tasksInfo);
        if (centralSinkJobRecord.count(late_job) == 0) {
            centralSinkJobRecord.insert(late_job);
            centralJobs.forwardJobs.push_back(late_job);
        }

        JobCEC jobSink = sf_fork.sink_job.GetJobWithinHyperPeriod(tasksInfo);
        if (centralSinkJobRecord.find(jobSink) == centralSinkJobRecord.end()) {
            centralSinkJobRecord.insert(jobSink);
            centralJobs.forwardJobs.push_back(jobSink);
        }
    }
    return centralJobs;
}

ActiveJobs FindActiveJobs(
    const CentralJobs &centralJobs, SFOrder &jobOrder,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    std::vector<ActiveJob> activeJobs;
    std::unordered_set<JobCEC> jobRecord;
    jobRecord.reserve(tasksInfo.length);

    for (uint i = 0; i < centralJobs.backwardJobs.size(); i++) {
        JobCEC jobCurr = centralJobs.backwardJobs[i];
        std::vector<JobCEC> backwardJobs = FindBackwardAdjacentJob(
            jobCurr, jobOrder, tasksInfo, startTimeVector);
        backwardJobs.push_back(jobCurr);
        for (uint j = 0; j < backwardJobs.size(); j++) {
            if (jobRecord.find(backwardJobs[j]) == jobRecord.end()) {
                jobRecord.insert(backwardJobs[j]);
                activeJobs.push_back(ActiveJob(backwardJobs[j], false));
            }
        }
    }
    for (uint i = 0; i < centralJobs.forwardJobs.size(); i++) {
        JobCEC jobCurr = centralJobs.forwardJobs[i];
        std::vector<JobCEC> forwardJobs = FindForwardAdjacentJob(
            jobCurr, jobOrder, tasksInfo, startTimeVector);
        forwardJobs.push_back(jobCurr);
        for (uint j = 0; j < forwardJobs.size(); j++) {
            if (jobRecord.find(forwardJobs[j]) == jobRecord.end()) {
                jobRecord.insert(forwardJobs[j]);
                activeJobs.push_back(ActiveJob(forwardJobs[j], true));
            }
        }
    }
    return ActiveJobs(activeJobs, jobRecord);
}

bool WhetherJobBreakChainRTDA(const JobCEC &job, LLint startP, LLint finishP,
                              const LongestCAChain &longestJobChains,
                              const DAG_Model &dagTasks, SFOrder &jobOrder,
                              const TaskSetInfoDerived &tasksInfo,
                              std::string obj_type) {
    if (obj_type == "ReactionTimeObj")
        return WhetherJobBreakChainRT(job, startP, finishP, longestJobChains,
                                      dagTasks, jobOrder, tasksInfo);
    else if (obj_type == "DataAgeObj")
        return WhetherJobBreakChainDA(job, startP, finishP, longestJobChains,
                                      dagTasks, jobOrder, tasksInfo);
    else
        CoutError("Unrecognized obj type in WhetherJobBreakChain!");
    return true;
}
bool JobChainContainTask(const std::vector<JobCEC> &jobChainCurr, int taskId) {
    for (uint i = 0; i < jobChainCurr.size(); i++) {
        if (jobChainCurr[i].taskId == taskId)
            return true;
    }
    return false;
}
// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove job, and then insert its
// start/finish instances at startP/finishP If you want to be safer, return more
// 'true' no need for profiler
bool WhetherJobBreakChainRT(const JobCEC &jobRelocate, LLint startP,
                            LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo) {
    SFOrder jobOrderNew = jobOrder;
    jobOrderNew.RemoveJob(jobRelocate);
    jobOrderNew.InsertStart(jobRelocate, startP);
    jobOrderNew.InsertFinish(jobRelocate, finishP);
    for (uint i = 0; i < longestJobChains.size(); i++) {
        const std::vector<JobCEC> &jobChainCurr =
            longestJobChains[i];  // iterate through each job chain;
        if (JobChainContainTask(jobChainCurr, jobRelocate.taskId)) {
            int siblingJobIndex =
                FindSiblingJobIndex(jobRelocate, jobChainCurr);
            if (siblingJobIndex == -1)
                continue;  // job doesn't appear in this taskChainCurr
            JobCEC sibJob = jobChainCurr[siblingJobIndex];
            if (sibJob.EqualWithinHyperPeriod(
                    jobRelocate,
                    tasksInfo))  // this may not be necessary, but is a safe
                                 // solution
                return true;

            if (siblingJobIndex == 0) {  // new source job may initiate a
                                         // different cause-effect chain
                // JobCEC afterSibJob =
                //     jobChainCurr[siblingJobIndex +
                //                  1];  // assume the length of the
                //                       // chain is longer than 1
                // if (sibJob.jobId < jobRelocate.jobId &&
                //     finishP <
                //         jobOrder.GetJobStartInstancePosition(afterSibJob)) {
                //     return true;
                // }
                continue;  // it seems like no possible situations will break
                           // the chain in this case
            } else {       // the job is not a source task's job
                if (sibJob.jobId < jobRelocate.jobId)
                    continue;  // the job cannot react earlier than sibJob,
                               // and so cannot change reaction relationship
                JobCEC sibJobImmediateSourceJob =
                    jobChainCurr[siblingJobIndex - 1];
                JobCEC first_react_job_in_new_job_order = FindFirstReactJob(
                    sibJobImmediateSourceJob, jobRelocate.taskId, jobOrderNew);
                if (first_react_job_in_new_job_order != sibJob)
                    return true;
            }
        }
    }

    return false;
}

bool WhetherJobBreakJobChainDA(const JobCEC &jobRelocate, LLint startP,
                               LLint finishP, SFOrder &jobOrder,
                               const TaskSetInfoDerived &tasksInfo,
                               SFOrder &jobOrderNew,
                               const std::vector<JobCEC> &jobChainCurr) {
    if (JobChainContainTask(jobChainCurr, jobRelocate.taskId)) {
        int siblingJobIndex = FindSiblingJobIndex(jobRelocate, jobChainCurr);
        if (siblingJobIndex == -1)
            return false;  // job doesn't appear in this taskChainCurr
        JobCEC sibJob = jobChainCurr[siblingJobIndex];
        if (sibJob.EqualWithinHyperPeriod(jobRelocate, tasksInfo))
            return true;

        if (siblingJobIndex ==
            jobChainCurr.size() - 1) {  // new source job may initiate a
                                        // different cause-effect chain
            // JobCEC beforeSibJob =
            //     jobChainCurr[siblingJobIndex -
            //                  1];  // assume the length of the chain is
            //                       // longer than 1
            // if (jobRelocate.jobId < sibJob.jobId &&
            //     jobOrder.GetJobFinishInstancePosition(beforeSibJob) <
            //         startP) {
            //     return true;
            // }
            return false;  // it seems like no possible situations will break
                           // the chain in this case
        } else {           // the job is not a source task's job
            if (jobRelocate.jobId < sibJob.jobId)
                return false;  // the job cannot finish later than sibJob,
                               // and so cannot change immediate backward
                               // job chain
            JobCEC sibJobImmediateFollowJob = jobChainCurr[siblingJobIndex + 1];
            JobCEC last_read_job_in_new_job_order =
                FindLastReadingJob(sibJobImmediateFollowJob, jobRelocate.taskId,
                                   jobOrderNew, tasksInfo);
            if (last_read_job_in_new_job_order != sibJob)
                return true;
        }
    }
    return false;
}

bool WhetherJobBreakChainDA(const JobCEC &jobRelocate, LLint startP,
                            LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo) {
    SFOrder jobOrderNew = jobOrder;
    jobOrderNew.RemoveJob(jobRelocate);
    jobOrderNew.InsertStart(jobRelocate, startP);
    jobOrderNew.InsertFinish(jobRelocate, finishP);
    for (uint i = 0; i < longestJobChains.size(); i++) {
        const std::vector<JobCEC> &jobChainCurr =
            longestJobChains[i];  // iterate through each job chain;
        bool whetherJobBreakJobChain =
            WhetherJobBreakJobChainDA(jobRelocate, startP, finishP, jobOrder,
                                      tasksInfo, jobOrderNew, jobChainCurr);
        if (whetherJobBreakJobChain)
            return true;
    }
    return false;
}

bool WhetherJobBreakChainSF(const JobCEC &jobRelocate, LLint startP,
                            LLint finishP, const WorstSF_JobFork &worst_sf_fork,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo) {
    SFOrder jobOrderNew = jobOrder;
    jobOrderNew.RemoveJob(jobRelocate);
    jobOrderNew.InsertStart(jobRelocate, startP);
    jobOrderNew.InsertFinish(jobRelocate, finishP);
    for (const SF_JobFork &sf_job_fork : worst_sf_fork.worst_fork_) {
        for (const JobCEC &job_source : sf_job_fork.source_jobs) {
            std::vector<JobCEC> jobChainCurr;
            jobChainCurr.reserve(2);
            jobChainCurr.push_back(job_source);
            jobChainCurr.push_back(sf_job_fork.sink_job);
            bool whetherJobBreakJobChain = WhetherJobBreakJobChainDA(
                jobRelocate, startP, finishP, jobOrder, tasksInfo, jobOrderNew,
                jobChainCurr);
            if (whetherJobBreakJobChain)
                return true;
        }
    }
    return false;
}

bool WhetherJobBreakChainRTDA_Fast(const JobCEC &jobRelocate,const LongestCAChain &longestJobChains,
                                   const DAG_Model &dagTasks, SFOrder &jobOrder,
                                   const TaskSetInfoDerived &tasksInfo,
                                   std::string obj_type ) {
    if (obj_type == "ReactionTimeObj" || obj_type == "DataAgeObj") {
        int breakCount = 0;
        for (uint i = 0; i < longestJobChains.size(); i++) {
            const std::vector<JobCEC> &jobChainCurr =
                longestJobChains[i];  // iterate through each job chain;
            if (JobChainContainTask(jobChainCurr, jobRelocate.taskId)) {
                int siblingJobIndex =
                    FindSiblingJobIndex(jobRelocate, jobChainCurr);
                if (siblingJobIndex == -1)
                    continue;  // job doesn't appear in this taskChainCurr
                JobCEC sibJob = jobChainCurr[siblingJobIndex];
                if (sibJob.DirectAdjacent(jobRelocate, tasksInfo)) // this might result in unsafe skip
                    if (++breakCount >= GlobalVariablesDAGOpt::breakChainThresholdIA)
                        return true;
            }
        }
        return false;
    }
    else
        CoutError("Unrecognized obj type in WhetherJobBreakChain!");
    return false;
}

bool WhetherJobBreakChainSF_Fast(const JobCEC &jobRelocate,
                                 const WorstSF_JobFork &worst_sf_fork,
                                 const DAG_Model &dagTasks, SFOrder &jobOrder,
                                 const TaskSetInfoDerived &tasksInfo) {
    int breakCount = 0;
    for (const SF_JobFork &sf_job_fork : worst_sf_fork.worst_fork_) {
        if (jobRelocate.taskId == sf_job_fork.sink_job.taskId) {
            if (jobRelocate.DirectAdjacent(sf_job_fork.sink_job, tasksInfo)) // this might result in unsafe skip
                if (++breakCount >= GlobalVariablesDAGOpt::breakChainThresholdIA)
                    return true;
        }
        for (const JobCEC &job_source : sf_job_fork.source_jobs) {
            if (jobRelocate.taskId == job_source.taskId) {
                if (jobRelocate.DirectAdjacent(job_source, tasksInfo)) // this might result in unsafe skip
                    if (++breakCount >= GlobalVariablesDAGOpt::breakChainThresholdIA)
                        return true;
            }
        }
    }
    return false;
}

}  // namespace OrderOptDAG_SPACE