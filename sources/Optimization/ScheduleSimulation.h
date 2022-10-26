#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Tools/colormod.h"
#include "sources/Optimization/JobGroups.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Tools/profilier.h"
#include "sources/Optimization/SFOrder.h"
namespace OrderOptDAG_SPACE
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
        size_t size() const { return taskQueue.size(); }
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
            uint lftJobIndex = -1;
            double lftAll = std::numeric_limits<double>::max();

            for (uint i = 0; i < taskQueue.size(); i++)
            {
                JobCEC jobCurr(taskQueue[i].first, taskQueue[i].second);
                double lft = GetDeadline(jobCurr, tasksInfo) - tasksInfo.tasks[jobCurr.taskId].executionTime;
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

    class EventPool
    {
    public:
        std::vector<LLint> events_;
        void Insert(LLint time)
        {
            if (find(events_.begin(), events_.end(), time) != events_.end())
                return;
            auto ite = std::upper_bound(events_.begin(), events_.end(), time);
            events_.insert(
                ite,
                time);
        }

        LLint PopMinEvent()
        {
            LLint event = events_[0];
            events_.erase(events_.begin());
            return event;
        }

        void Print()
        {
            for (size_t i = 0; i < events_.size(); i++)
            {
                std::cout << events_[i] << ", ";
            }
            std::cout << std::endl;
        }
        size_t size() const { return events_.size(); }
    };

    void AddTasksToRunQueue(RunQueue &runQueue, const TaskSet &tasks, LLint timeNow, EventPool &eventPool)
    {
        // check whether to add new instances
        for (uint i = 0; i < tasks.size(); i++)
        {
            if (timeNow % tasks[i].period == 0)
            {
                runQueue.insert({i, timeNow / tasks[i].period});
                eventPool.Insert(timeNow + tasks[i].period);
            }
        }
    }

    void UpdateSTVAfterPopTask(RunQueue::ID_INSTANCE_PAIR &job, VectorDynamic &startTimeVector,
                               LLint timeNow, vector<LLint> &nextFree, const TaskSet &tasks,
                               vector<bool> &busy, int processorId, vector<LLint> &sizeOfVariables)
    {
        int id = job.first;
        LLint instance_id = job.second;
        LLint index_overall = IndexTran_Instance2Overall(id, instance_id, sizeOfVariables);
        startTimeVector(index_overall, 0) = timeNow;
        nextFree[processorId] = timeNow + tasks[id].executionTime;
        busy[processorId] = true;
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
    VectorDynamic SimulateFixedPrioritySched(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, LLint timeInitial = 0)
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

        for (LLint timeNow = timeInitial; timeNow < tasksInfo.hyperPeriod; timeNow++)
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

    // exam all the jobs in jobOrder_ have been dispatched
    bool ExamPrecedenceJobSatisfied(JobCEC jobCurr, std::vector<LLint> &jobScheduled, const JobOrderMultiCore &jobOrder)
    {
        for (uint i = 0; i < jobOrder.jobIndexMap_.at(jobCurr); i++)
        {
            if (jobScheduled[i] == -1) // prior job has not been dispatched yet
            {
                return false;
            }
        }
        return true;
    }

    // exam all the jobs in strictPrecedenceMap_ have been finished
    bool ExamPrecedenceJobSatisfiedNP(JobCEC jobCurr, LLint currTime, const JobOrderMultiCore &jobOrder, std::vector<LLint> &jobScheduled, const TaskSetInfoDerived &tasksInfo)
    {
        if (jobOrder.HaveSerialConstraint(jobCurr) == false)
            return true;
        for (uint i = 0; i < jobOrder.jobIndexMap_.at(jobCurr); i++)
        {
            const JobCEC &jobPrec = jobOrder.jobOrder_.at(i);
            if (jobOrder.HaveSerialConstraint(jobPrec) == false)
            {
                continue;
            }
            LLint jobPrecIndex = jobOrder.jobIndexMap_.at(jobPrec);
            if (jobScheduled[jobPrecIndex] == -1 || jobScheduled[jobPrecIndex] + tasksInfo.tasks[jobPrec.taskId].executionTime > currTime)
                return false;
        }
        return true;
    }

    /**
     * @brief
     */
    RunQueue::ID_INSTANCE_PAIR PopTaskLS(RunQueue &runQueue, const JobOrderMultiCore &jobOrder,
                                         LLint timeNow, std::vector<LLint> &jobScheduled, const TaskSetInfoDerived &tasksInfo)
    {
        BeginTimerAppInProfiler;
        std::vector<RunQueue::ID_INSTANCE_PAIR> &taskQueue = runQueue.taskQueue;
        if (taskQueue.empty())
            CoutError("TaskQueue is empty!");

        double leastIndex = std::numeric_limits<double>::max();
        LLint leastIndexJobInQueue = -1;
        // take all the tasks:
        std::unordered_set<JobCEC> set;
        for (uint i = 0; i < taskQueue.size(); i++)
        {
            JobCEC jobCurr(taskQueue[i].first, taskQueue[i].second);
            if (ExamPrecedenceJobSatisfiedNP(jobCurr, timeNow, jobOrder, jobScheduled, tasksInfo) && ExamPrecedenceJobSatisfied(jobCurr, jobScheduled, jobOrder))
            {
                LLint priority = jobOrder.jobIndexMap_.at(jobCurr);
                if (priority < leastIndex)
                {
                    leastIndex = priority;
                    leastIndexJobInQueue = i;
                }
            }
        }
        RunQueue::ID_INSTANCE_PAIR jobPop = std::make_pair(-1, -1);
        if (leastIndexJobInQueue == -1)
        {
            return jobPop;
        }
        jobPop = taskQueue[leastIndexJobInQueue];
        taskQueue.erase(taskQueue.begin() + leastIndexJobInQueue);

        EndTimerAppInProfiler;
        return jobPop;
    };

    // TODO: when two jobs have same priority, choose the one with higher precedence priority
    VectorDynamic ListSchedulingLFTPA(DAG_Model &dagTasks,
                                      TaskSetInfoDerived &tasksInfo, int processorNum, const std::optional<JobOrderMultiCore> &jobOrder = std::nullopt,
                                      boost::optional<std::vector<uint> &> processorIdVec = boost::none)
    {
        BeginTimerAppInProfiler;
        const TaskSet &tasks = dagTasks.tasks;
        VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

        // contains the index of tasks to run
        RunQueue runQueue(tasks);

        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);

        std::vector<LLint> jobScheduled(jobOrder ? ((*jobOrder).size()) : 0, -1); // order is the same as JobOrder's jobs
        EventPool eventPool;
        eventPool.Insert(0);
        LLint timeInitial = eventPool.PopMinEvent();
        if (processorIdVec)
            *processorIdVec = Eigen2Vector<uint>(initial);

        for (LLint timeNow = timeInitial; timeNow < tasksInfo.hyperPeriod;)
        {
            AddTasksToRunQueue(runQueue, tasks, timeNow, eventPool);

            for (int processorId = 0; processorId < processorNum; processorId++)
            {
                if (timeNow >= nextFree[processorId])
                {
                    busy[processorId] = false;
                }
                if (!busy[processorId] && (!runQueue.empty()))
                {
                    RunQueue::ID_INSTANCE_PAIR p;
                    bool findTaskToSchedule;
                    if (jobOrder)
                    {
                        findTaskToSchedule = false;
                        while (!findTaskToSchedule && runQueue.taskQueue.size() > 0)
                        {
                            p = PopTaskLS(runQueue, *jobOrder, timeNow, jobScheduled, tasksInfo);
                            if (p.first == -1)
                            {
                                break;
                            }
                            else
                            {
                                JobCEC jobCurr(p.first, p.second);
                                jobScheduled[jobOrder->jobIndexMap_.at(jobCurr)] = timeNow;
                                findTaskToSchedule = true;
                            }
                        }
                    }
                    else
                    {
                        p = runQueue.popLeastFinishTime(tasksInfo);
                        findTaskToSchedule = true;
                    }
                    if (findTaskToSchedule)
                    {
                        UpdateSTVAfterPopTask(p, initial, timeNow, nextFree, tasks, busy, processorId, tasksInfo.sizeOfVariables);
                        eventPool.Insert(nextFree[processorId]);
                        if (processorIdVec)
                        {
                            (*processorIdVec)[IndexTran_Instance2Overall(p.first, p.second, tasksInfo.sizeOfVariables)] = processorId;
                            dagTasks.tasks[p.first].processorId = processorId;
                        }
                    }
                }
            }
            timeNow = eventPool.PopMinEvent();
        }
        if (runQueue.size() != 0)
        {
            initial = GenerateVectorDynamic(tasksInfo.variableDimension);
        }
        EndTimerAppInProfiler;
        return initial;
    }

    class SchedulingAlgorithm
    {
    public:
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, int processorNum, JobOrderMultiCore &jobOrder)
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
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, int processorNum, const std::optional<JobOrderMultiCore> &jobOrder = std::nullopt,
                                      boost::optional<std::vector<uint> &> processorIdVec = boost::none)
        {
            return ListSchedulingLFTPA(dagTasks, tasksInfo, 1, jobOrder, processorIdVec);
        }
    };

    // list scheduling whose task assignment is decided by scheduling algorithms
    class LSchedulingFreeTA : public SchedulingAlgorithm
    {
    public:
        static VectorDynamic Schedule(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, int processorNum, const std::optional<JobOrderMultiCore> &jobOrder = std::nullopt,
                                      boost::optional<std::vector<uint> &> processorIdVec = boost::none)
        {
            return ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, jobOrder, processorIdVec);
        }
    };

    VectorDynamic SFOrderScheduling(DAG_Model &dagTasks,
                                    TaskSetInfoDerived &tasksInfo, int processorNum, SFOrder &jobOrder,
                                    boost::optional<std::vector<uint> &> processorIdVec = boost::none)
    {
        BeginTimerAppInProfiler;
        const TaskSet &tasks = dagTasks.tasks;
        std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

        VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);
        vector<LLint> scheduledFinishTime(tasksInfo.variableDimension, -1);

        if (processorIdVec)
            *processorIdVec = Eigen2Vector<uint>(startTimeVector);

        LLint timeNow = 0;
        for (auto &currentInstance : instanceOrder)
        {
            if (currentInstance.type == 'f')
            {
                if (scheduledFinishTime[GetJobUniqueId(currentInstance.job, tasksInfo)] < timeNow)
                {
                    startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
                    break;
                }
                else
                {
                    timeNow = scheduledFinishTime[GetJobUniqueId(currentInstance.job, tasksInfo)];
                }
            }
            else if (currentInstance.type == 's')
            {
                if (timeNow < GetActivationTime(currentInstance.job, tasksInfo))
                {
                    timeNow = GetActivationTime(currentInstance.job, tasksInfo);
                }
                for (int processorId = 0; processorId < processorNum; processorId++)
                {
                    if (timeNow >= nextFree[processorId])
                    {
                        busy[processorId] = false;
                    }
                    if (!busy[processorId])
                    {
                        LLint uniqueJobId = GetJobUniqueId(currentInstance.job, tasksInfo);
                        startTimeVector(uniqueJobId, 0) = timeNow;
                        nextFree[processorId] = timeNow + tasks[currentInstance.job.taskId].executionTime;
                        scheduledFinishTime[uniqueJobId] = nextFree[processorId];
                        busy[processorId] = true;
                        if (processorIdVec)
                        {
                            (*processorIdVec)[uniqueJobId] = processorId;
                        }
                        break;
                    }
                }
            }
            else
            {
                if (debugMode)
                {
                    std::cout << "Failed to schedule SFOrder: Unknown TimeInstance type.\n";
                }
                startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
                break;
            }
        }

        EndTimerAppInProfiler;
        return startTimeVector;
    }
} // namespace OrderOptDAG_SPACE
