#pragma once
#include "unordered_set"
#include "unordered_map"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/InitialEstimate.h"

namespace DAG_SPACE
{

    class JobOrder
    {
    public:
        TaskSetInfoDerived tasksInfo_;
        std::vector<JobCEC> jobOrder_;
        std::unordered_map<JobCEC, LLint> jobOrderMap_;

        JobOrder() {}
        JobOrder(TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo)
        {
            std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector = ObtainAllJobSchedule(tasksInfo, startTimeVector);
            timeJobVector = SortJobSchedule(timeJobVector);
            jobOrder_.reserve(timeJobVector.size());
            for (LLint i = 0; i < static_cast<LLint>(timeJobVector.size()); i++)
            {
                jobOrder_.push_back(timeJobVector[i].second);
            }
            UpdateMap();
        }

        void UpdateMap()
        {
            for (LLint i = 0; i < static_cast<LLint>(jobOrder_.size()); i++)
            {
                jobOrderMap_[jobOrder_[i]] = i;
            }
        }

        void print()
        {
            for (uint i = 0; i < jobOrder_.size(); i++)
            {
                std::cout << jobOrder_[i].ToString() + ", ";
                if (i % 4 == 0)
                    std::cout << std::endl;
            }
        }

        size_t size() { return jobOrder_.size(); }

        JobCEC operator[](LLint index) { return jobOrder_[index]; }

        // jobIndex and newPosition is relative to the original job vector
        void ChangeJobOrder(LLint jobIndex, LLint newPosition)
        {
            if (jobIndex == newPosition)
                return;
            else if (jobIndex < 0 || jobIndex >= static_cast<LLint>(jobOrder_.size()) || newPosition < 0 || newPosition >= static_cast<LLint>(jobOrder_.size()))
            {
                CoutError("Index out-of-range in ChangeJobOrder");
            }

            JobCEC job = jobOrder_[jobIndex];
            jobOrder_.erase(jobOrder_.begin() + jobIndex);
            jobOrder_.insert(jobOrder_.begin() + newPosition, job);
            UpdateMap();
        }
    };

    RunQueue::ID_INSTANCE_PAIR PopTaskLS(RunQueue &runQueue, JobOrder &jobOrder)
    {
        std::vector<RunQueue::ID_INSTANCE_PAIR> &taskQueue = runQueue.taskQueue;
        if (taskQueue.empty())
            CoutError("TaskQueue is empty!");
        uint lftJobIndex;
        double lftAll = std::numeric_limits<double>::max();

        for (uint i = 0; i < taskQueue.size(); i++)
        {
            JobCEC currJob(taskQueue[i].first, taskQueue[i].second);
            LLint priority = jobOrder.jobOrderMap_[currJob];
            if (priority < lftAll)
            {
                lftJobIndex = i;
                lftAll = priority;
            }
        }
        RunQueue::ID_INSTANCE_PAIR jobPop = taskQueue[lftJobIndex];
        taskQueue.erase(taskQueue.begin() + lftJobIndex);
        return jobPop;
    };
    VectorDynamic ListSchedulingGivenOrder(const DAG_Model &dagTasks,
                                           vector<LLint> &sizeOfVariables,
                                           int variableDimension,
                                           JobOrder &jobOrder,
                                           LLint currTime = 0)
    {

        int N = dagTasks.tasks.size();
        const TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);
        LLint hyperPeriod = HyperPeriod(tasks);

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
        int processorNum = processorTaskSet.size();
        // it maps from tasks[i].processorId to index in runQueues&busy&nextFree
        ProcessorId2Index processorId2Index = CreateProcessorId2Index(tasks);
        // contains the index of tasks to run
        vector<RunQueue> runQueues;
        runQueues.reserve(processorNum);
        for (int i = 0; i < processorNum; i++)
        {
            runQueues.push_back(RunQueue(tasks));
        }

        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);
        // LLint nextFree;

        for (LLint timeNow = currTime; timeNow < hyperPeriod; timeNow++)
        {

            // check whether to add new instances
            for (int i = 0; i < N; i++)
            {
                if (timeNow % tasks[i].period == 0)
                {
                    int currId = tasks[i].processorId;
                    runQueues[processorId2Index[currId]].insert({i, timeNow / tasks[i].period});
                }
            }
            for (int i = 0; i < processorNum; i++)
            {
                if (timeNow >= nextFree[i])
                {
                    busy[i] = false;
                }
                if (!busy[i] && (!runQueues[i].empty()))
                {
                    auto sth = PopTaskLS(runQueues[i], jobOrder);
                    // TODO: strictly follow JobOrder
                    int id = sth.first;
                    LLint instance_id = sth.second;
                    LLint index_overall = IndexTran_Instance2Overall(id, instance_id, sizeOfVariables);
                    initial(index_overall, 0) = timeNow;
                    nextFree[i] = timeNow + tasks[id].executionTime;
                    busy[i] = true;
                }
            }
        }

        return initial;
    }

    // TODO: merge code!!!
    // with Processor Assignment
    VectorDynamic ListSchedulingGivenOrderPA(const DAG_Model &dagTasks,
                                             JobOrder &jobOrder,
                                             LLint currTime = 0)
    {
        int N = dagTasks.tasks.size();
        const TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);
        LLint hyperPeriod = HyperPeriod(tasks);

        int processorNum = coreNumberAva;
        // contains the index of tasks to run
        RunQueue runQueue(tasks);

        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);

        for (LLint timeNow = currTime; timeNow < hyperPeriod; timeNow++)
        {

            // check whether to add new instances
            for (int i = 0; i < N; i++)
            {
                if (timeNow % tasks[i].period == 0)
                {
                    int currId = tasks[i].processorId;
                    runQueue.insert({i, timeNow / tasks[i].period});
                }
            }
            for (int i = 0; i < processorNum; i++)
            {
                if (timeNow >= nextFree[i])
                {
                    busy[i] = false;
                }
                if (!busy[i] && (!runQueue.empty()))
                {
                    auto sth = PopTaskLS(runQueue, jobOrder);
                    int id = sth.first;
                    LLint instance_id = sth.second;
                    LLint index_overall = IndexTran_Instance2Overall(id, instance_id, tasksInfo.sizeOfVariables);
                    initial(index_overall, 0) = timeNow;
                    nextFree[i] = timeNow + tasks[id].executionTime;
                    busy[i] = true;
                }
            }
        }

        return initial;
    }

} // namespace DAG_SPACE