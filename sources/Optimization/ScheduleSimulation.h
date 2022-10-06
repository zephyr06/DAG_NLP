#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Tools/colormod.h"
#include "sources/Optimization/JobGroups.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/JobOrder.h"

namespace DAG_SPACE
{

    typedef std::map<int, int> ProcessorId2Index;
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

    void AddTasksToRunQueues(vector<RunQueue> &runQueues, const TaskSet &tasks, ProcessorId2Index &processorId2Index, LLint timeNow)
    {
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (timeNow % tasks[i].period == 0)
            {
                int currId = tasks[i].processorId;
                runQueues[processorId2Index[currId]].insert({i, timeNow / tasks[i].period});
            }
        }
    }

    void AddTasksToRunQueue(RunQueue &runQueue, const TaskSet &tasks, LLint timeNow)
    {
        // check whether to add new instances
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (timeNow % tasks[i].period == 0)
            {
                runQueue.insert({i, timeNow / tasks[i].period});
            }
        }
    }

    void UpdateSTVAfterPopTask(RunQueue::ID_INSTANCE_PAIR &job, VectorDynamic &startTimeVector,
                               LLint timeNow, vector<LLint> &nextFree, const TaskSet &tasks,
                               vector<bool> &busy, int processodId, vector<LLint> &sizeOfVariables)
    {
        int id = job.first;
        LLint instance_id = job.second;
        LLint index_overall = IndexTran_Instance2Overall(id, instance_id, sizeOfVariables);
        startTimeVector(index_overall, 0) = timeNow;
        nextFree[processodId] = timeNow + tasks[id].executionTime;
        busy[processodId] = true;
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
    VectorDynamic SimulateFixedPrioritySched(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, LLint currTime = 0)
    {
        const TaskSet &tasks = dagTasks.tasks;
        VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

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

        for (LLint timeNow = currTime; timeNow < tasksInfo.hyperPeriod; timeNow++)
        {
            // check whether to add new instances
            AddTasksToRunQueues(runQueues, tasks, processorId2Index, timeNow);

            for (int i = 0; i < processorNum; i++)
            {
                if (timeNow >= nextFree[i])
                {
                    busy[i] = false;
                }
                if (timeNow >= nextFree[i] && (!runQueues[i].empty()))
                {
                    auto sth = runQueues[i].pop();
                    UpdateSTVAfterPopTask(sth, initial, timeNow, nextFree, tasks, busy, i, tasksInfo.sizeOfVariables);
                }
            }
        }

        return initial;
    }

    RunQueue::ID_INSTANCE_PAIR PopTaskLS(RunQueue &runQueue, const JobOrder &jobOrder)
    {
        std::vector<RunQueue::ID_INSTANCE_PAIR> &taskQueue = runQueue.taskQueue;
        if (taskQueue.empty())
            CoutError("TaskQueue is empty!");
        uint lftJobIndex;
        double lftAll = std::numeric_limits<double>::max();

        for (uint i = 0; i < taskQueue.size(); i++)
        {
            JobCEC currJob(taskQueue[i].first, taskQueue[i].second);
            LLint priority = jobOrder.jobOrderMap_.at(currJob);
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

    // TODO: when two jobs have same priority, choose the one with higher precedence priority
    VectorDynamic ListSchedulingLFTPA(const DAG_Model &dagTasks,
                                      TaskSetInfoDerived &tasksInfo, int processorNum, const std::optional<JobOrder> &jobOrder = std::nullopt)
    {
        const TaskSet &tasks = dagTasks.tasks;
        VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

        // contains the index of tasks to run
        RunQueue runQueue(tasks);

        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);

        std::vector<bool> jobScheduled(jobOrder ? ((*jobOrder).size()) : 0, false); // order is the same as JobOrder's jobs
        LLint currTime = 0;
        for (LLint timeNow = currTime; timeNow < tasksInfo.hyperPeriod; timeNow++)
        {
            AddTasksToRunQueue(runQueue, tasks, timeNow);
            for (int i = 0; i < processorNum; i++)
            {
                if (timeNow >= nextFree[i])
                {
                    busy[i] = false;
                }
                if (!busy[i] && (!runQueue.empty()))
                {
                    RunQueue::ID_INSTANCE_PAIR p;
                    if (jobOrder)
                    {
                        p = PopTaskLS(runQueue, *jobOrder);
                        JobCEC jobCurr(p.first, p.second);
                        // efficiency can be improved
                        for (uint j = 0; j < jobOrder->jobOrderMap_.at(jobCurr); j++)
                        {
                            if (!jobScheduled[j]) // prior job has not been scheduled yet
                            {
                                runQueue.insert({jobCurr.taskId, jobCurr.jobId});
                                break;
                            }
                        }
                        jobScheduled[jobOrder->jobOrderMap_.at(jobCurr)] = true;
                    }
                    else
                    {
                        p = runQueue.popLeastFinishTime(tasksInfo);
                    }

                    UpdateSTVAfterPopTask(p, initial, timeNow, nextFree, tasks, busy, i, tasksInfo.sizeOfVariables);
                }
            }
        }

        return initial;
    }

    class SchedulingAlgorithm
    {
    public:
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, JobOrder &jobOrder)
        {
            CoutError("Base function in SchedulingAlgorithm must be overwritten!");
            return GenerateVectorDynamic1D(0);
        }
    };
    // list scheduling with known task assignment
    class LSchedulingKnownTA : public SchedulingAlgorithm
    {
    public:
        // If used, this function needs to be carefully checked!
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, JobOrder &jobOrder, int processorNum)
        {
            return ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder);
        }
    };

    // list scheduling whose task assignment is decided by scheduling algorithms
    class LSchedulingFreeTA : public SchedulingAlgorithm
    {
    public:
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, JobOrder &jobOrder, int processorNum)
        {
            return ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, jobOrder);
        }
    };
} // namespace DAG_SPACE

// VectorDynamic ListSchedulingLFT(const DAG_Model &dagTasks,
//                                 TaskSetInfoDerived &tasksInfo)
// {
//     const TaskSet &tasks = dagTasks.tasks;
//     VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

//     ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
//     int processorNum = processorTaskSet.size();
//     // it maps from tasks[i].processorId to index in runQueues&busy&nextFree
//     ProcessorId2Index processorId2Index = CreateProcessorId2Index(tasks);
//     // contains the index of tasks to run
//     vector<RunQueue> runQueues;
//     runQueues.reserve(processorNum);
//     for (int i = 0; i < processorNum; i++)
//     {
//         runQueues.push_back(RunQueue(tasks));
//     }

//     vector<bool> busy(processorNum, false);
//     vector<LLint> nextFree(processorNum, -1);
//     // LLint nextFree;
//     LLint currTime = 0;
//     for (LLint timeNow = currTime; timeNow < tasksInfo.hyperPeriod; timeNow++)
//     {

//         // check whether to add new instances
//         AddTasksToRunQueues(runQueues, tasks, processorId2Index, timeNow);

//         for (int i = 0; i < processorNum; i++)
//         {
//             if (timeNow >= nextFree[i])
//             {
//                 busy[i] = false;
//             }
//             if (timeNow >= nextFree[i] && (!runQueues[i].empty()))
//             {
//                 auto sth = runQueues[i].popLeastFinishTime(tasksInfo);
//                 UpdateSTVAfterPopTask(sth, initial, timeNow, nextFree, tasks, busy, i, tasksInfo.sizeOfVariables);
//             }
//         }
//     }

//     return initial;
// }

// VectorDynamic ListSchedulingGivenOrder(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo,
//                                        JobOrder &jobOrder)
// {
//     const TaskSet &tasks = dagTasks.tasks;
//     VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

//     ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
//     int processorNum = processorTaskSet.size();
//     // it maps from tasks[i].processorId to index in runQueues&busy&nextFree
//     ProcessorId2Index processorId2Index = CreateProcessorId2Index(tasks);
//     // contains the index of tasks to run
//     vector<RunQueue> runQueues;
//     runQueues.reserve(processorNum);
//     for (int i = 0; i < processorNum; i++)
//     {
//         runQueues.push_back(RunQueue(tasks));
//     }

//     vector<bool> busy(processorNum, false);
//     vector<LLint> nextFree(processorNum, -1);
//     std::vector<bool> jobScheduled(jobOrder.size(), false); // order is the same as JobOrder's jobs
//     std::vector<bool> jobFinished(jobOrder.size(), false);  // order is the same as JobOrder's jobs

//     LLint currTime = 0;
//     for (LLint timeNow = currTime; timeNow < tasksInfo.hyperPeriod; timeNow++)
//     {
//         AddTasksToRunQueues(runQueues, tasks, processorId2Index, timeNow);

//         for (int i = 0; i < processorNum; i++)
//         {
//             if (timeNow >= nextFree[i])
//             {
//                 busy[i] = false;
//             }
//             if (!busy[i] && (!runQueues[i].empty()))
//             {
//                 auto sth = PopTaskLS(runQueues[i], jobOrder);
//                 JobCEC jobCurr(sth.first, sth.second);
//                 // this part can improve efficiency
//                 for (uint j = 0; j < jobOrder.jobOrderMap_[jobCurr]; j++)
//                 {
//                     if (!jobScheduled[j]) // prior job has not been scheduled yet
//                     {
//                         runQueues[processorId2Index[tasks[jobCurr.taskId].processorId]].insert({jobCurr.taskId, jobCurr.jobId});
//                         break;
//                     }
//                 }
//                 // TODO:
//                 // for(JobCEC jobMustBeFinished: jobOrder.strictPrecedenceMap_[jobCurr])
//                 // {
//                 //     if (!jobFinished[jobOrder.jobOrderMap_[jobMustBeFinished]])
//                 //     {
//                 //         runQueues[processorId2Index[tasks[jobCurr.taskId].processorId]].insert({jobCurr.taskId, jobCurr.jobId});
//                 //         break;
//                 //     }
//                 // }

//                 UpdateSTVAfterPopTask(sth, initial, timeNow, nextFree, tasks, busy, i, tasksInfo.sizeOfVariables);
//                 jobScheduled[jobOrder.jobOrderMap_[jobCurr]] = true;
//             }
//         }
//     }

//     return initial;
// }

// with Processor Assignment
// VectorDynamic ListSchedulingGivenOrderPA(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, int processorNum, JobOrder &jobOrder)
// {
//     const TaskSet &tasks = dagTasks.tasks;
//     VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

//     // contains the index of tasks to run
//     RunQueue runQueue(tasks);

//     vector<bool> busy(processorNum, false);
//     vector<LLint> nextFree(processorNum, -1);
//     std::vector<bool> jobScheduled(jobOrder.size(), false); // order is the same as JobOrder's jobs
//     LLint currTime = 0;
//     for (LLint timeNow = currTime; timeNow < tasksInfo.hyperPeriod; timeNow++)
//     {
//         AddTasksToRunQueue(runQueue, tasks, timeNow);
//         for (int i = 0; i < processorNum; i++)
//         {
//             if (timeNow >= nextFree[i])
//             {
//                 busy[i] = false;
//             }
//             if (!busy[i] && (!runQueue.empty()))
//             {
//                 auto sth = PopTaskLS(runQueue, jobOrder);
//                 JobCEC jobCurr(sth.first, sth.second);
//                 // efficiency can be improved
//                 for (uint j = 0; j < jobOrder.jobOrderMap_[jobCurr]; j++)
//                 {
//                     if (!jobScheduled[j]) // prior job has not been scheduled yet
//                     {
//                         runQueue.insert({jobCurr.taskId, jobCurr.jobId});
//                         break;
//                     }
//                 }
//                 UpdateSTVAfterPopTask(sth, initial, timeNow, nextFree, tasks, busy, i, tasksInfo.sizeOfVariables);
//                 jobScheduled[jobOrder.jobOrderMap_[jobCurr]] = true;
//             }
//         }
//     }

//     return initial;
// }
