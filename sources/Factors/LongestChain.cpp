#include "sources/Factors/LongestChain.h"

#include <unordered_set>

#include "sources/Utils/profilier.h"
namespace OrderOptDAG_SPACE {

// sourceJob is not used because it is equivalent as jobChain[0]
double GetReactionTime(const std::vector<JobCEC> &jobChain,
                       const VectorDynamic &startTimeVector,
                       const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
    if (jobChain.size() <= 1) {
        CoutError(
            "Cannot analyze reaction time because jobChain is too short!");
    };
    return GetFinishTime(jobChain.back(), startTimeVector, tasksInfo) -
           GetStartTime(jobChain[0], startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>> GetMaxReactionTimeChains(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
    const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo, double tolerance) {
    std::vector<std::vector<JobCEC>> longestChains;
    longestChains.reserve(
        10);  // there should be no more than 10 longest chains
    double maxLength = -2;
    for (const auto &pair : react_chain_map) {
        // const JobCEC &jobCurr = pair.first;
        const std::vector<JobCEC> &jobChain = pair.second;
        double lengthChainCurr =
            GetReactionTime(jobChain, startTimeVector, tasksInfo);
        if (std::abs(lengthChainCurr - maxLength) < tolerance) {
            longestChains.push_back(jobChain);
        } else if (lengthChainCurr > maxLength) {
            longestChains.clear();
            longestChains.push_back(jobChain);
            maxLength = lengthChainCurr;
        }
    }
    return longestChains;
}

double GetDataAge(const std::vector<JobCEC> &jobChain,
                  const VectorDynamic &startTimeVector,
                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo) {
    if (jobChain.size() <= 1) {
        CoutError(
            "Cannot analyze reaction time because jobChain is too short!");
    };

    JobCEC sinkJob = jobChain.back();
    JobCEC sourceJob = jobChain[0];
    return GetFinishTime(sinkJob, startTimeVector, tasksInfo) -
           GetStartTime(sourceJob, startTimeVector, tasksInfo);
}

std::vector<std::vector<JobCEC>> GetMaxDataAgeChains(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &da_chain_map,
    const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo, double tolerance) {
    std::vector<std::vector<JobCEC>> longestChains;
    longestChains.reserve(
        10);  // there should be no more than 10 longest chains
    double maxLength = -2;
    for (const auto &pair : da_chain_map) {
        const JobCEC &jobCurr = pair.first;
        const std::vector<JobCEC> &jobChain = pair.second;

        double lengthChainCurr =
            GetDataAge(jobChain, startTimeVector, tasksInfo);
        if (std::abs(lengthChainCurr - maxLength) < tolerance) {
            longestChains.push_back(jobChain);
        } else if (lengthChainCurr > maxLength) {
            longestChains.clear();
            longestChains.push_back(jobChain);
            maxLength = lengthChainCurr;
        }
    }
    return longestChains;
}

std::vector<std::vector<JobCEC>> LongestCAChain::GetLongestJobChains_RT(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum) {
    std::vector<std::vector<JobCEC>> longestChains;
    longestChains.reserve(dagTasks.chains_.size() *
                          3);  // 3x is usually not necessary, though

    for (uint i = 0; i < dagTasks.chains_.size(); i++) {
        auto react_chain_map =
            GetReactionChainMap(dagTasks, tasksInfo, jobOrder, processorNum,
                                dagTasks.chains_[i], i);
        auto chains = GetMaxReactionTimeChains(react_chain_map, startTimeVector,
                                               tasksInfo);
        for (uint i = 0; i < chains.size(); i++)
            longestChains.push_back(chains[i]);
    }
    return longestChains;
}

std::vector<std::vector<JobCEC>> LongestCAChain::GetLongestJobChains_DA(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum) {
    std::vector<std::vector<JobCEC>> longestChains;
    longestChains.reserve(dagTasks.chains_.size() *
                          3);  // 3x is usually not necessary, though

    for (uint i = 0; i < dagTasks.chains_.size(); i++) {
        auto da_chain_map =
            GetDataAgeChainMap(dagTasks, tasksInfo, jobOrder, processorNum,
                               dagTasks.chains_[i], i);
        auto chains =
            GetMaxDataAgeChains(da_chain_map, startTimeVector, tasksInfo);
        for (uint i = 0; i < chains.size(); i++)
            longestChains.push_back(chains[i]);
    }
    return longestChains;
}

std::vector<std::vector<JobCEC>> LongestCAChain::GetLongestJobChains_SF(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum) {
    return {{}};
}

std::vector<std::vector<JobCEC>> LongestCAChain::FindLongestCAChain(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum) {
    if (obj_type_trait_ == "ReactionTimeObj")
        return GetLongestJobChains_RT(dagTasks, tasksInfo, jobOrder,
                                      startTimeVector, processorNum);
    else if (obj_type_trait_ == "DataAgeObj")
        return GetLongestJobChains_DA(dagTasks, tasksInfo, jobOrder,
                                      startTimeVector, processorNum);
    // else if (obj_type_trait_ == "SensorFusionObj")
    //   return GetLongestJobChains_DA(dagTasks, tasksInfo, jobOrder,
    //   startTimeVector, processorNum);
    else
        CoutError("Unrecognized obj type in FindLongestCAChain");
    return {{}};
}

// no need for profiler
// std::vector<std::vector<JobCEC>> LongestCAChain::FindLongestCAChain(const
// DAG_Model &dagTasks,
//                                                                     const
//                                                                     TaskSetInfoDerived
//                                                                     &tasksInfo,
//                                                                     SFOrder
//                                                                     &jobOrder,
//                                                                     const
//                                                                     VectorDynamic
//                                                                     &startTimeVector,
//                                                                     int
//                                                                     processorNum)
// {
//   std::vector<std::vector<JobCEC>> longestChains;
//   longestChains.reserve(dagTasks.chains_.size() * 3); // 3x is usually not
//   necessary, though

//   std::unordered_set<JobCEC> sourceJobRecords;

//   for (uint i = 0; i < dagTasks.chains_.size(); i++)
//   {
//     auto react_chain_map =
//         GetReactionChainMap(dagTasks, tasksInfo, jobOrder, processorNum,
//         dagTasks.chains_[i], i);
//     auto chains = GetMaxReactionTimeChains(react_chain_map, startTimeVector,
//     tasksInfo); auto chains2 = GetMaxDataAgeChains(react_chain_map,
//     startTimeVector, tasksInfo); chains.insert(chains.end(), chains2.begin(),
//     chains2.end()); for (uint i = 0; i < chains.size(); i++)
//     {
//       // if (sourceJobRecords.find(chains[i][0]) == sourceJobRecords.end()) {
//       longestChains.push_back(chains[i]);
//       //   sourceJobRecords.insert(chains[i][0]);
//       // }
//     }
//   }
//   return longestChains;
// }

int FindSiblingJobIndex(const JobCEC &job,
                        const std::vector<JobCEC> &jobChainCurr) {
    for (uint i = 0; i < jobChainCurr.size(); i++) {
        if (job.taskId == jobChainCurr[i].taskId) {
            return i;
        }
    }
    // CoutError("Didn't find sibling job in FindSiblingJobIndex");
    return -1;
}

bool WhetherJobBreakChain(const JobCEC &job, LLint startP, LLint finishP,
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
    for (auto &taskChainCurr : dagTasks.chains_) {
        auto itr = std::find(taskChainCurr.begin(), taskChainCurr.end(),
                             jobRelocate.taskId);
        if (itr != taskChainCurr.end()) {
            for (uint i = 0; i < longestJobChains.size(); i++) {
                const std::vector<JobCEC> &jobChainCurr =
                    longestJobChains[i];  // iterate through each job chain;
                int siblingJobIndex =
                    FindSiblingJobIndex(jobRelocate, jobChainCurr);
                if (siblingJobIndex == -1)
                    continue;  // job doesn't appear in this taskChainCurr
                JobCEC sibJob = jobChainCurr[siblingJobIndex];
                if (sibJob.EqualWithinHyperPeriod(
                        jobRelocate,
                        tasksInfo))  // this may not be necessary, but is a safe
                                     // solution
                {
                    return true;
                }

                if (siblingJobIndex == 0) {  // new source job may initiate a
                                             // different cause-effect chain
                    JobCEC afterSibJob =
                        jobChainCurr[siblingJobIndex +
                                     1];  // assume the length of the chain is
                                          // longer than 1
                    if (sibJob.jobId < jobRelocate.jobId &&
                        finishP <
                            jobOrder.GetJobStartInstancePosition(afterSibJob)) {
                        return true;
                    }
                } else {  // the job is not a source task's job
                    if (sibJob.jobId < jobRelocate.jobId)
                        continue;  // the job cannot react earlier than sibJob,
                                   // and so cannot change reaction relationship
                    JobCEC sibJobImmediateSourceJob =
                        jobChainCurr[siblingJobIndex - 1];
                    JobCEC first_react_job_in_new_job_order =
                        FindFirstReactJob(sibJobImmediateSourceJob,
                                          jobRelocate.taskId, jobOrderNew);
                    if (first_react_job_in_new_job_order != sibJob)
                        return true;
                }
            }
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
    for (auto &taskChainCurr : dagTasks.chains_) {
        auto itr = std::find(taskChainCurr.begin(), taskChainCurr.end(),
                             jobRelocate.taskId);
        if (itr != taskChainCurr.end()) {
            for (uint i = 0; i < longestJobChains.size(); i++) {
                const std::vector<JobCEC> &jobChainCurr =
                    longestJobChains[i];  // iterate through each job chain;
                int siblingJobIndex =
                    FindSiblingJobIndex(jobRelocate, jobChainCurr);
                if (siblingJobIndex == -1)
                    continue;  // job doesn't appear in this taskChainCurr
                JobCEC sibJob = jobChainCurr[siblingJobIndex];
                if (sibJob.EqualWithinHyperPeriod(jobRelocate, tasksInfo))
                    return true;

                if (siblingJobIndex ==
                    taskChainCurr.size() -
                        1) {  // new source job may initiate a different
                              // cause-effect chain
                    JobCEC beforeSibJob =
                        jobChainCurr[siblingJobIndex -
                                     1];  // assume the length of the chain is
                                          // longer than 1
                    if (jobRelocate.jobId < sibJob.jobId &&
                        jobOrder.GetJobFinishInstancePosition(beforeSibJob) <
                            startP) {
                        return true;
                    }
                } else {  // the job is not a source task's job
                    if (jobRelocate.jobId < sibJob.jobId)
                        continue;  // the job cannot finish later than sibJob,
                                   // and so cannot change immediate backward
                                   // job chain
                    JobCEC sibJobImmediateFollowJob =
                        jobChainCurr[siblingJobIndex + 1];
                    JobCEC last_read_job_in_new_job_order = FindLastReadingJob(
                        sibJobImmediateFollowJob, jobRelocate.taskId,
                        jobOrderNew, tasksInfo);
                    if (last_read_job_in_new_job_order != sibJob)
                        return true;
                }
            }
        }
    }
    return false;
}

std::unordered_map<JobCEC, int> ExtractIndependentJobGroups(
    const SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo) {
#ifdef PROFILE_CODE
    BeginTimer("ExtractIndependentJobGroups");
#endif
    std::unordered_map<JobCEC, int> jobGroupMap;
    int jobGroupIndex = 0;
    jobGroupMap.insert({jobOrder.at(0).job, jobGroupIndex});
    std::unordered_set<JobCEC> jobsNotMeetFinish;
    jobsNotMeetFinish.reserve(tasksInfo.length);

    for (uint i = 1; i < jobOrder.size(); i++) {
        const TimeInstance &instCurr = jobOrder.at(i);
        const TimeInstance &instPrev = jobOrder.at(i - 1);
        // if (instPrev.job == instCurr.job)
        //   continue;

        if (GetDeadline(instPrev.job, tasksInfo) <=
            GetActivationTime(instCurr.job, tasksInfo)) {
            bool maxOfAll = true;
            for (auto itr = jobsNotMeetFinish.begin();
                 itr != jobsNotMeetFinish.end(); itr++) {
                JobCEC jobIte = *itr;
                if (GetDeadline(jobIte, tasksInfo) >
                    GetActivationTime(instCurr.job, tasksInfo)) {
                    maxOfAll = false;
                    break;
                }
            }
            if (maxOfAll)
                jobGroupIndex++;
        }
        if (jobGroupMap.find(instCurr.job) == jobGroupMap.end())
            jobGroupMap.insert({instCurr.job, jobGroupIndex});

        if (instCurr.type == 's') {
            jobsNotMeetFinish.insert(instCurr.job);
        } else {
            jobsNotMeetFinish.erase(instCurr.job);
        }
    }
#ifdef PROFILE_CODE
    EndTimer("ExtractIndependentJobGroups");
#endif
    // std::cout << "The number of job groups in ExtractIndependentJobGroups: "
    // << jobGroupIndex << std::endl;
    return jobGroupMap;
}

bool ExamMaxStartChange(LLint jobChangedOldStart, LLint jobChangedOldFinish,
                        LLint jobChangedNewStart, LLint jobChangedNewfinish,
                        LLint jobCurrOldStart, LLint jobCurrOldFinish,
                        LLint jobCurrNewStart, LLint jobCurrNewFinish) {
    // exam start constraints;
    if (jobChangedOldStart > jobCurrOldStart) {
        if (jobChangedNewStart < jobCurrNewStart)
            return true;
    }
    if (jobChangedOldFinish > jobCurrOldStart) {
        if (jobChangedNewfinish < jobCurrNewStart)
            return true;
    }

    // exam finish constraints;
    if (jobChangedOldStart > jobCurrOldFinish) {
        if (jobChangedNewStart < jobCurrNewFinish)
            return true;
    }
    if (jobChangedOldFinish > jobCurrOldFinish) {
        if (jobChangedNewfinish < jobCurrNewFinish)
            return true;
    }

    return false;
}

// if the constraints for jobCurr.start/jobCurr.finish's upper bound change,
// then there is possibly an influence
bool WhetherInfluenceJobSource(
    JobCEC jobCurr, const JobCEC &jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    LLint startP, LLint finishP,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
        return false;
    if (jobCurr.EqualWithinHyperPeriod(jobChanged, tasksInfo))
        return true;
    if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) -
                 GetActivationTime(jobCurr, tasksInfo)) < 1e-3)
        return false;
    LLint jobChangedOldStart = jobOrder.GetJobStartInstancePosition(jobChanged);
    LLint jobChangedOldFinish =
        jobOrder.GetJobFinishInstancePosition(jobChanged);
    LLint jobCurrOldStart = jobOrder.GetJobStartInstancePosition(jobCurr);
    LLint jobCurrOldFinish = jobOrder.GetJobFinishInstancePosition(jobCurr);

    JobPosition jobCurrNewPosition(jobCurrOldStart, jobCurrOldFinish);
    // jobOrder.RemoveJob(jobChanged);
    jobCurrNewPosition.UpdateAfterRemoveInstance(jobChangedOldFinish);
    jobCurrNewPosition.UpdateAfterRemoveInstance(jobChangedOldStart);
    // jobOrder.InsertStart(jobChanged, startP);
    jobCurrNewPosition.UpdateAfterInsertInstance(startP);
    // jobOrder.InsertFinish(jobChanged, finishP);
    jobCurrNewPosition.UpdateAfterInsertInstance(finishP);
    LLint jobCurrNewStart = jobCurrNewPosition.start_;
    LLint jobCurrNewFinish = jobCurrNewPosition.finish_;

    // jobOrder.RemoveJob(jobChanged);
    // jobOrder.InsertStart(jobChanged, startP);
    // jobOrder.InsertFinish(jobChanged, finishP);
    // LLint jobCurrNewStart = jobOrder.GetJobStartInstancePosition(jobCurr);
    // LLint jobCurrNewFinish = jobOrder.GetJobFinishInstancePosition(jobCurr);

    if (ExamMaxStartChange(jobChangedOldStart, jobChangedOldFinish, startP,
                           finishP, jobCurrOldStart, jobCurrOldFinish,
                           jobCurrNewStart, jobCurrNewFinish))
        return true;

    return false;
}

bool ExamMinStartChange(LLint jobChangedOldStart, LLint jobChangedOldFinish,
                        LLint jobChangedNewStart, LLint jobChangedNewfinish,
                        LLint jobCurrOldStart, LLint jobCurrOldFinish,
                        LLint jobCurrNewStart, LLint jobCurrNewFinish) {
    // exam start
    if (jobChangedOldFinish < jobCurrOldStart) {
        if (jobChangedNewfinish > jobCurrNewStart)
            return true;
    }
    if (jobChangedOldStart < jobCurrOldStart) {
        if (jobChangedNewStart > jobCurrNewStart)
            return true;
    }

    // exam finish
    if (jobChangedOldFinish < jobCurrOldFinish) {
        if (jobChangedNewfinish > jobCurrNewFinish)
            return true;
    }
    if (jobChangedOldStart < jobCurrOldFinish) {
        if (jobChangedNewStart > jobCurrNewFinish)
            return true;
    }
    return false;
}

// if the constraints for jobCurr.start/jobCurr.finish's lower bound change,
// then there is possibly an influence
bool WhetherInfluenceJobSink(
    JobCEC jobCurr, const JobCEC &jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    LLint startP, LLint finishP,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector) {
    jobCurr = jobCurr.GetJobWithinHyperPeriod(tasksInfo);
    if (!WhetherInfluenceJobSimple(jobCurr, jobChanged, jobGroupMap))
        return false;
    if (jobCurr.EqualWithinHyperPeriod(jobChanged, tasksInfo))
        return true;
    if (std::abs(GetStartTime(jobCurr, startTimeVector, tasksInfo) +
                 GetExecutionTime(jobCurr, tasksInfo) -
                 GetDeadline(jobCurr, tasksInfo)) < 1e-3)
        return false;
    LLint jobChangedOldStart = jobOrder.GetJobStartInstancePosition(jobChanged);
    LLint jobChangedOldFinish =
        jobOrder.GetJobFinishInstancePosition(jobChanged);
    LLint jobCurrOldStart = jobOrder.GetJobStartInstancePosition(jobCurr);
    LLint jobCurrOldFinish = jobOrder.GetJobFinishInstancePosition(jobCurr);

    JobPosition jobCurrNewPosition(jobCurrOldStart, jobCurrOldFinish);
    // jobOrder.RemoveJob(jobChanged);
    jobCurrNewPosition.UpdateAfterRemoveInstance(jobChangedOldFinish);
    jobCurrNewPosition.UpdateAfterRemoveInstance(jobChangedOldStart);
    // jobOrder.InsertStart(jobChanged, startP);
    jobCurrNewPosition.UpdateAfterInsertInstance(startP);
    // jobOrder.InsertFinish(jobChanged, finishP);
    jobCurrNewPosition.UpdateAfterInsertInstance(finishP);
    LLint jobCurrNewStart = jobCurrNewPosition.start_;
    LLint jobCurrNewFinish = jobCurrNewPosition.finish_;

    // jobOrder.RemoveJob(jobChanged);
    // jobOrder.InsertStart(jobChanged, startP);
    // jobOrder.InsertFinish(jobChanged, finishP);
    // LLint jobCurrNewStart = jobOrder.GetJobStartInstancePosition(jobCurr);
    // LLint jobCurrNewFinish = jobOrder.GetJobFinishInstancePosition(jobCurr);

    if (ExamMinStartChange(jobChangedOldStart, jobChangedOldFinish, startP,
                           finishP, jobCurrOldStart, jobCurrOldFinish,
                           jobCurrNewStart, jobCurrNewFinish))
        return true;

    return false;
}
}  // namespace OrderOptDAG_SPACE