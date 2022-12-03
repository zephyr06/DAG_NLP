#pragma once
#ifndef JOBCEC_H
#define JOBECE_H

#include "unordered_map"

// #include "gtsam/base/Value.h"

#include "sources/TaskModel/RegularTasks.h"

namespace OrderOptDAG_SPACE
{
    struct JobCEC
    {
        int taskId;
        LLint jobId;
        JobCEC() : taskId(-1), jobId(0) {}
        JobCEC(int taskId, LLint jobId) : taskId(taskId), jobId(jobId) {}
        JobCEC(std::pair<int, LLint> p) : taskId(p.first), jobId(p.second) {}

        bool operator==(const JobCEC &other) const
        {
            return taskId == other.taskId && jobId == other.jobId;
        }
        bool operator!=(const JobCEC &other) const
        {
            return !(*this == other);
        }

        std::string ToString() const
        {
            return "T" + std::to_string(taskId) + "_" + std::to_string(jobId);
        }
    };

    double GetStartTime(JobCEC jobCEC, const gtsam::Values &x, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

    inline double GetDeadline(JobCEC job, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[job.taskId].period * job.jobId + tasksInfo.tasks[job.taskId].deadline;
    }

    inline double GetActivationTime(JobCEC job, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[job.taskId].period * job.jobId;
    }

    double GetStartTime(JobCEC jobCEC, const VectorDynamic &x, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

    inline double GetFinishTime(JobCEC jobCEC, const gtsam::Values &x, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }
    inline double GetFinishTime(JobCEC jobCEC, const VectorDynamic &x, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }

    std::vector<std::pair<std::pair<double, double>, JobCEC>> ObtainAllJobSchedule(const RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &x);

    std::vector<std::pair<std::pair<double, double>, JobCEC>> SortJobSchedule(
        std::vector<std::pair<std::pair<double, double>, JobCEC>> &timeJobVector);

    void PrintSchedule(const RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &x);

    // map the job to the first hyper period and return job's unique id
    LLint GetJobUniqueId(const JobCEC &jobCEC, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

    JobCEC GetJobCECFromUniqueId(LLint id, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

    inline double GetExecutionTime(LLint id, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[GetJobCECFromUniqueId(id, tasksInfo).taskId].executionTime;
    }
    inline double GetExecutionTime(const JobCEC &jobCEC, const RegularTaskSystem::TaskSetInfoDerived &tasksInfo)
    {
        // return tasksInfo.tasks[GetJobCECFromUniqueId(GetJobUniqueId(jobCEC, tasksInfo), tasksInfo).taskId].executionTime;
        return tasksInfo.tasks[jobCEC.taskId].executionTime;
    }
}

template <>
struct std::hash<OrderOptDAG_SPACE::JobCEC>
{
    std::size_t operator()(const OrderOptDAG_SPACE::JobCEC &jobCEC) const
    {
        std::string str = std::to_string(jobCEC.taskId) + ", " + std::to_string(jobCEC.jobId);
        return std::hash<std::string>{}(str);
    }
};

#endif