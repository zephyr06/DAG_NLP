#pragma once
#ifndef JOBCEC_H
#define JOBECE_H

#include "unordered_map"

#include "gtsam/base/Value.h"

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

    double GetStartTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo.N)
        {
            CoutError("GetStartTime receives invalid jobCEC!");
        }
        int jobNumInHyperPeriod = tasksInfo.hyperPeriod / tasksInfo.tasks[jobCEC.taskId].period;

        double res = x.at<VectorDynamic>(GenerateKey(jobCEC.taskId, jobCEC.jobId % jobNumInHyperPeriod))(0) + jobCEC.jobId / jobNumInHyperPeriod * tasksInfo.hyperPeriod;
        return res;
    }

    inline double GetDeadline(JobCEC job, const TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[job.taskId].period * job.jobId + tasksInfo.tasks[job.taskId].deadline;
    }
    inline double GetActivationTime(JobCEC job, const TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[job.taskId].period * job.jobId;
    }

    double GetStartTime(JobCEC jobCEC, const VectorDynamic &x, const TaskSetInfoDerived &tasksInfo)
    {
        if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo.N)
        {
            CoutError("GetStartTime receives invalid jobCEC!");
        }
        int jobNumInHyperPeriod = tasksInfo.hyperPeriod / tasksInfo.tasks[jobCEC.taskId].period;

        double res = x(IndexTran_Instance2Overall(jobCEC.taskId, jobCEC.jobId % jobNumInHyperPeriod, tasksInfo.sizeOfVariables)) + jobCEC.jobId / jobNumInHyperPeriod * tasksInfo.hyperPeriod;
        return res;
    }

    inline double GetFinishTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }
    inline double GetFinishTime(JobCEC jobCEC, const VectorDynamic &x, const TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }

    std::vector<std::pair<std::pair<double, double>, JobCEC>> ObtainAllJobSchedule(TaskSetInfoDerived &tasksInfo, VectorDynamic &x)
    {
        // souted start time
        std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector;
        int temp_count = 0;
        for (LLint task_id = 0; task_id < LLint(tasksInfo.sizeOfVariables.size()); task_id++)
        {
            auto execution_time = tasksInfo.tasks[task_id].executionTime;
            for (LLint job_id = 0; job_id < tasksInfo.sizeOfVariables[task_id]; job_id++)
            {
                double start_time = x(temp_count, 0);
                double finish_time = x(temp_count, 0) + execution_time;
                std::pair<double, double> time_pair(start_time, finish_time);
                JobCEC job_pair(task_id, job_id);
                timeJobVector.push_back(std::make_pair(time_pair, job_pair));
                temp_count++;
            }
        }
        return timeJobVector;
    }

    std::vector<std::pair<std::pair<double, double>, JobCEC>> SortJobSchedule(
        std::vector<std::pair<std::pair<double, double>, JobCEC>> &timeJobVector)
    {

        std::sort(timeJobVector.begin(), timeJobVector.end(),
                  [](auto a, auto b)
                  {
                      return a.first.first < b.first.first;
                  });
        return timeJobVector;
    }

    void PrintSchedule(TaskSetInfoDerived &tasksInfo, VectorDynamic &x)
    {
        auto timeJobVector = ObtainAllJobSchedule(tasksInfo, x);
        timeJobVector = SortJobSchedule(timeJobVector);
        // std::cout << "Current sorted start time: " << std::endl;

        int temp_count = 0;
        for (auto time_job_pair : timeJobVector)
        {
            // std::cout << std::setprecision(6);
            std::cout << "T" << time_job_pair.second.taskId << "_" << time_job_pair.second.jobId << " (" << time_job_pair.first.first << ", ";
            std::cout << time_job_pair.first.second << "), ";
            temp_count++;
            if (temp_count % 4 == 0)
            {
                std::cout << std::endl;
            }
            // std::cout << std::setprecision(8);
        }
        std::cout << "\n************************************************" << std::endl;
    }

    // map the job to the first hyper period and return job's unique id
    LLint GetJobUniqueId(const JobCEC &jobCEC, const TaskSetInfoDerived &tasksInfo)
    {
        LLint id = jobCEC.jobId % tasksInfo.sizeOfVariables[jobCEC.taskId];
        for (int i = 0; i < jobCEC.taskId; i++)
        {
            id += tasksInfo.sizeOfVariables[i];
        }
        return id;
    }

    JobCEC GetJobCECFromUniqueId(LLint id, const TaskSetInfoDerived &tasksInfo)
    {
        if (id < 0 || id >= tasksInfo.variableDimension)
        {
            return JobCEC();
        }
        int task_id = 0;
        for (auto size : tasksInfo.sizeOfVariables)
        {
            if (id < size)
            {
                return JobCEC(task_id, id);
            }
            else
            {
                id -= size;
                task_id++;
            }
        }
        return JobCEC();
    }

    double GetExecutionTime(LLint id, const TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[GetJobCECFromUniqueId(id, tasksInfo).taskId].executionTime;
    }
    double GetExecutionTime(JobCEC &jobCEC, const TaskSetInfoDerived &tasksInfo)
    {
        return tasksInfo.tasks[GetJobCECFromUniqueId(GetJobUniqueId(jobCEC, tasksInfo), tasksInfo).taskId].executionTime;
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