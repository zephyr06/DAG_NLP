/**
 * @file SensorFusionFactor.h
 * @author Sen Wang
 * @brief This file implements sensor fusion factor for non-preemptive case;
 * @version 0.1
 * @date 2022-06-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once
#include "sources/TaskModel/DAG_Model.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE {
using namespace RegularTaskSystem;
LLint CountSFError(const DAG_Model &dagTasks,
                   const std::vector<LLint> &sizeOfVariables);

struct IndexData {
    LLint index;
    double time;
};

struct SF_JobFork {
    SF_JobFork() {}
    SF_JobFork(JobCEC sink_job, std::vector<JobCEC> source_jobs)
        : sink_job(sink_job), source_jobs(source_jobs) {}

    // data members
    JobCEC sink_job;
    std::vector<JobCEC> source_jobs;
};

struct EarliestAndLatestFinishedJob {
    EarliestAndLatestFinishedJob() {}
    EarliestAndLatestFinishedJob(JobCEC early, JobCEC late)
        : early_job(early), late_job(late) {}

    inline double GetFinishTimeDiff(const VectorDynamic &startTimeVector,
                                    const TaskSetInfoDerived &tasksInfo) {
        return GetFinishTime(late_job, startTimeVector, tasksInfo) -
               GetFinishTime(early_job, startTimeVector, tasksInfo);
    }
    JobCEC early_job;
    JobCEC late_job;
};
EarliestAndLatestFinishedJob FindEarlyAndLateJob(
    const SF_JobFork &job_fork, const VectorDynamic &startTimeVector,
    const TaskSetInfoDerived &tasksInfo);

std::pair<int, int> ExtractMaxDistance(
    std::vector<IndexData> &sourceFinishTime);

inline double ExtractMaxDistance(std::vector<double> &sourceFinishTime) {
    return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
           *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
}

VectorDynamic ObtainSensorFusionError(const DAG_Model &dagTasks,
                                      const TaskSetInfoDerived &tasksInfo,
                                      const VectorDynamic &startTimeVector);

std::unordered_map<JobCEC, std::vector<JobCEC>> GetSensorMapFromSingleJob(
    const TaskSetInfoDerived &tasksInfo, const int task_id,
    const TaskSet &precede_tasks, const VectorDynamic &x);

}  // namespace OrderOptDAG_SPACE