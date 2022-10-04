#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Tools/colormod.h"
#include "sources/Optimization/JobGroups.h"
#include "sources/Utils/JobCEC.h"

namespace DAG_SPACE
{
    class RunQueue
    {
    public:
        typedef std::pair<int, LLint> ID_INSTANCE_PAIR; // taskId, jobId
        std::vector<RegularTaskSystem::Task> tasks;
        int N;
        std::vector<ID_INSTANCE_PAIR> taskQueue;
        // ProcessorId2Index processorId2Index;,
        //                    ProcessorId2Index &processorId2Index                                             processorId2Index(processorId2Index)
        /**
         * @brief Construct a new Run Queue object
         *
         * @param tasks : all the tasks in the DAG model rather than all the tasks assigned to a single processor
         */
        RunQueue(TaskSet tasks) : tasks(tasks)
        {
            N = tasks.size();
            taskQueue.reserve(N);
        }
        /**
         * @brief
         *  always maintain taskQueue's inverse priority order, i.e. first task has highest priority,
         * highest priority means highest priority value by number
         *
         * @param id_instance : pair:
         * first: task index (task id)
         * second: instance number
         */
        void insert(ID_INSTANCE_PAIR id_instance)
        {
            if (taskQueue.size() == 0)
                taskQueue.push_back(id_instance);
            else
            {
                LLint id = id_instance.first;
                // LLint instance = id_instance.second;
                LLint prev = FindPrev(id);
                taskQueue.insert(taskQueue.begin() + prev, id_instance);
            }
        }
        /**
         * @brief Given a new task's id, find its position in the queue
         * such that the queue is ordered from highest priority to lowest priority
         *
         * @param id: new task's id
         * @return LLint
         */
        LLint FindPrev(LLint id)
        {
            Task taskCurr = tasks[id];
            double priorityCurr = taskCurr.priority();

            LLint left = 0;
            LLint right = taskQueue.size();
            while (left < right)
            {
                LLint mid = (left + right) / 2;
                double priorityMid = tasks[taskQueue[mid].first].priority();
                if (priorityMid == priorityCurr)
                {
                    return mid;
                }
                else if (priorityCurr < priorityMid)
                {
                    left = mid + 1;
                }
                else
                {
                    right = mid;
                }
            }
            return left;
        }

        /**
         * @brief Given a task's id, remove this task from the runQueue
         *
         * @param id
         */
        void erase(LLint id)
        {
            for (size_t i = 0; i < taskQueue.size(); i++)
            {
                if (taskQueue[i].first == id)
                {
                    taskQueue.erase(taskQueue.begin() + i);
                    return;
                }
            }
            CoutError("The element to erase is not found!");
        }
        /**
         * @brief return the front task's id in the taskQueue
         *
         * @return LLint
         */
        LLint front()
        {
            if (taskQueue.empty())
                CoutError("TaskQueue is empty!");

            return taskQueue[0].first;
        }
        /**
         * @brief return the front task's id in the taskQueue,
         * and remove it from taskQueue
         *
         * @return LLint
         */
        ID_INSTANCE_PAIR pop()
        {
            if (taskQueue.empty())
                CoutError("TaskQueue is empty!");
            ID_INSTANCE_PAIR first = taskQueue[0];
            taskQueue.erase(taskQueue.begin());
            return first;
        }
        bool empty()
        {
            return taskQueue.empty();
        }

        ID_INSTANCE_PAIR popLeastFinishTime(TaskSetInfoDerived &tasksInfo)
        {
            if (taskQueue.empty())
                CoutError("TaskQueue is empty!");
            uint lftJobIndex;
            double lftAll = std::numeric_limits<double>::max();

            for (uint i = 0; i < taskQueue.size(); i++)
            {
                JobCEC currJob(taskQueue[i].first, taskQueue[i].second);
                double lft = GetDeadline(currJob, tasksInfo) - tasksInfo.tasks[currJob.taskId].executionTime;
                if (lft < lftAll)
                {
                    lftJobIndex = i;
                    lftAll = lft;
                }
            }
            ID_INSTANCE_PAIR jobPop = taskQueue[lftJobIndex];
            taskQueue.erase(taskQueue.begin() + lftJobIndex);
            return jobPop;
        }
    };
    /**
     * @brief Create a ProcessorId2Index object, only used in initialization estimation
     *
     * @return ProcessorId2Index: map, processorID to processorIdIndex
     */
    ProcessorId2Index CreateProcessorId2Index(const TaskSet &tasks)
    {
        ProcessorId2Index processorId2Index;

        int indexP = 0;
        int N = tasks.size();
        for (int i = 0; i < N; i++)
        {
            if (processorId2Index.find(tasks.at(i).processorId) == processorId2Index.end())
            {
                processorId2Index[tasks.at(i).processorId] = indexP++;
            }
        }
        return processorId2Index;
    }
    /**
     * @brief Warning! All the task sets must have int type values, otherwise it may generate inappropriate initialization method;
     * If you need to use double type, please use timeScaleFactor to transform it into int type with some acceptable accuracy.
     *
     * By default, priority is given by RM?
     * @param dagTasks
     * @param sizeOfVariables
     * @param variableDimension
     * @return VectorDynamic
     */
    VectorDynamic SimulateFixedPrioritySched(const DAG_Model &dagTasks,
                                             vector<LLint> &sizeOfVariables,
                                             int variableDimension, LLint currTime = 0)
    {
        int N = dagTasks.tasks.size();
        const TaskSet &tasks = dagTasks.tasks;
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
                    auto sth = runQueues[i].pop();
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

    VectorDynamic ListSchedulingLFT(const DAG_Model &dagTasks,
                                    vector<LLint> &sizeOfVariables,
                                    int variableDimension, LLint currTime = 0)
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
                    auto sth = runQueues[i].popLeastFinishTime(tasksInfo);
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
} // namespace DAG_SPACE