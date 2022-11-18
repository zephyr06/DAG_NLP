#pragma once
#include "unordered_map"

#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Utils/colormod.h"
#include "sources/Utils/JobCEC.h"
// #include "sources/Optimization/JobOrder.h"
// #include "sources/Utils/profilier.h"
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

        ID_INSTANCE_PAIR popLeastFinishTime(const TaskSetInfoDerived &tasksInfo, DAG_Model &dagTasks, double modifyPriorityBasedOnPrecedence = 0);
    };
    /**
     * @brief Create a ProcessorId2Index object, only used in initialization estimation
     *
     * @return ProcessorId2Index: map, processorID to processorIdIndex
     */
    ProcessorId2Index CreateProcessorId2Index(const TaskSet &tasks);

    void AddTasksToRunQueues(std::vector<RunQueue> &runQueues, const TaskSet &tasks, ProcessorId2Index &processorId2Index, LLint timeNow);

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

    void AddTasksToRunQueue(RunQueue &runQueue, const TaskSet &tasks, LLint timeNow, EventPool &eventPool);

    void UpdateSTVAfterPopTask(RunQueue::ID_INSTANCE_PAIR &job, VectorDynamic &startTimeVector,
                               LLint timeNow,std::vector<LLint> &nextFree, const TaskSet &tasks,
                              std::vector<bool> &busy, int processorId,std::vector<LLint> &sizeOfVariables);

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
    VectorDynamic SimulateFixedPrioritySched(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, LLint timeInitial = 0);

    // TODO: when two jobs have same priority, choose the one with higher precedence priority
    VectorDynamic ListSchedulingLFTPA(DAG_Model &dagTasks,
                                      TaskSetInfoDerived &tasksInfo, int processorNum,
                                      //   const std::optional<JobOrderMultiCore> &jobOrder = std::nullopt,
                                      boost::optional<std::vector<uint> &> processorIdVec = boost::none);

    VectorDynamic SFOrderScheduling(DAG_Model &dagTasks,
                                    const TaskSetInfoDerived &tasksInfo, int processorNum, const SFOrder &jobOrder,
                                    boost::optional<std::vector<uint> &> processorIdVec = boost::none);
} // namespace OrderOptDAG_SPACE
