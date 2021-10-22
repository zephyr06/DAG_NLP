
#pragma once
#include "unordered_map"

#include "RegularTasks.h"
#include "DAG_Model.h"
#include "GraphUtilsFromBGL.h"
#include "colormod.h"
typedef std::unordered_map<int, int> ProcessorId2Index;
using namespace RegularTaskSystem;
namespace DAG_SPACE
{

    /**
     * @brief Generate initial solution for the whole optimization
     * 
     * @param tasks 
     * @param sizeOfVariables 
     * @return VectorDynamic size (N+1), first N is start time for nodes, the last one is r.h.s.
     */
    VectorDynamic GenerateInitialForDAG_IndexMode(DAG_Model &dagTasks, vector<LLint> &sizeOfVariables, int variableDimension)
    {
        int N = dagTasks.tasks.size();
        TaskSet &tasks = dagTasks.tasks;
        vector<int> order = FindDependencyOrder(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);

        // m maps from tasks index to startTimeVector index
        std::unordered_map<int, LLint> m;
        LLint sumVariableNum = 0;
        for (int i = 0; i < N; i++)
        {
            m[i] = sumVariableNum;
            sumVariableNum += sizeOfVariables[i];
        }

        LLint index = 0;
        LLint currTime = 0;

        for (int i = 0; i < N; i++)
        {
            int currTaskIndex = order[i];
            for (int j = 0; j < sizeOfVariables[currTaskIndex]; j++)
            {
                initial(m[currTaskIndex] + j, 0) = j * tasks[currTaskIndex].period + index++;
            }
        }
        return initial;
    }
    VectorDynamic GenerateInitialForDAG_RelativeStart(DAG_Model &dagTasks,
                                                      vector<LLint> &sizeOfVariables,
                                                      int variableDimension)
    {
        int N = dagTasks.tasks.size();
        TaskSet &tasks = dagTasks.tasks;
        vector<int> order = FindDependencyOrder(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);

        LLint index = 0;
        LLint currTime = 0;
        vector<int> relativeStart;
        relativeStart.reserve(N);
        // schedule first instance for each DAG
        for (int i = 0; i < N; i++)
        {
            int currTaskIndex = order[i];

            for (int j = 0; j < sizeOfVariables[currTaskIndex]; j++)
            {
                LLint index_overall = IndexTran_Instance2Overall(currTaskIndex, j, sizeOfVariables);
                initial(index_overall, 0) = j * tasks[currTaskIndex].period + currTime;
            }
            currTime += tasks[currTaskIndex].executionTime;
        }
        return initial;
    }

    class RunQueue
    {
    public:
        typedef pair<int, LLint> ID_INSTANCE_PAIR;
        vector<RegularTaskSystem::Task> tasks;
        int N;
        vector<ID_INSTANCE_PAIR> taskQueue;
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
                LLint instance = id_instance.second;
                LLint prev = FindPrev(id);
                taskQueue.insert(taskQueue.begin() + prev, id_instance);
            }
        }
        /**
         * @brief Given a new task's id, find its position in the queue
         * such that the queue is order from highest priority to lowest priority
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
    };
    /**
     * @brief Warning! All the tasks's processorId must begin with 0, otherwise it reports Segmentation error.
     * 
     * @param dagTasks 
     * @param sizeOfVariables 
     * @param variableDimension 
     * @return VectorDynamic 
     */
    VectorDynamic GenerateInitial_RM(DAG_Model &dagTasks,
                                     vector<LLint> &sizeOfVariables,
                                     int variableDimension, LLint currTime = 0)
    {
        int N = dagTasks.tasks.size();
        TaskSet &tasks = dagTasks.tasks;
        vector<int> order = FindDependencyOrder(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);
        LLint hyperPeriod = HyperPeriod(tasks);

        LLint index = 0;
        vector<int> relativeStart;
        relativeStart.reserve(N);

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
        int processorNum = processorTaskSet.size();
        // it maps from tasks[i].processorId to index in runQueues&busy&nextFree

        ProcessorId2Index processorId2Index;
        int indexP = 0;
        for (int i = 0; i < N; i++)
        {
            if (processorId2Index.find(tasks[i].processorId) == processorId2Index.end())
            {
                processorId2Index[tasks[i].processorId] = indexP++;
            }
        }
        // contains the index of tasks to run
        vector<RunQueue> runQueues;
        runQueues.reserve(processorNum);
        for (int i = 0; i < processorNum; i++)
        {
            runQueues.push_back(RunQueue(tasks));
        }
        // RunQueue runQueue(tasks);
        // bool busy = false;
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
                if (timeNow == nextFree[i])
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
    /**
     * @brief Warning! All the tasks's processorId must begin with 0, otherwise it reports Segmentation error.
     * 
     * @param dagTasks 
     * @param sizeOfVariables 
     * @param variableDimension 
     * @return VectorDynamic 
     */
    VectorDynamic GenerateInitialForDAG_RM_DAG(DAG_Model &dagTasks,
                                               vector<LLint> &sizeOfVariables,
                                               int variableDimension)
    {
        int N = dagTasks.tasks.size();
        TaskSet &tasks = dagTasks.tasks;
        vector<int> order = FindDependencyOrder(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);
        LLint hyperPeriod = HyperPeriod(tasks);

        LLint index = 0;
        LLint currTime = 0;
        vector<int> relativeStart;
        relativeStart.reserve(N);

        for (int i = 0; i < N; i++)
        {
            int currTaskIndex = order[i];
            LLint index_overall = IndexTran_Instance2Overall(currTaskIndex, 0, sizeOfVariables);
            initial(index_overall, 0) = currTime;
            currTime += tasks[currTaskIndex].executionTime;
        }
        // schedule first instance for each DAG
        // TODO: if tasks' cumulative execution time is long,
        // it's possible to be larger than one task's period,
        // and so prevents generating appropriate initialization value for it.
        for (int i = 0; i < N; i++)
        {
            if (currTime > tasks[i].period)
            {
                for (int j = 1; j <= currTime / tasks[i].period; j++)
                {
                    LLint index_overall = IndexTran_Instance2Overall(i, j, sizeOfVariables);
                    initial(index_overall, 0) = (j)*tasks[i].period;
                }
            }
        }

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
        int processorNum = processorTaskSet.size();
        // it maps from tasks[i].processorId to index in runQueues&busy&nextFree

        ProcessorId2Index processorId2Index;
        int indexP = 0;
        for (int i = 0; i < N; i++)
        {
            if (processorId2Index.find(tasks[i].processorId) == processorId2Index.end())
            {
                processorId2Index[tasks[i].processorId] = indexP++;
            }
        }
        // contains the index of tasks to run
        vector<RunQueue> runQueues;
        runQueues.reserve(processorNum);
        for (int i = 0; i < processorNum; i++)
        {
            // TaskSet tasksCurr;
            // for (int j : processorTaskSet[i])
            // {
            //     tasksCurr.push_back(tasks[j]);
            // }
            runQueues.push_back(RunQueue(tasks));
        }
        // RunQueue runQueue(tasks);
        // bool busy = false;
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
                if (timeNow == nextFree[i])
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

    /**
     * @brief Generate initial estimate based on provided options'
     * param: initializeMethod: global parameter passed implicitly
     * 
     * @param dagTasks 
     * @param sizeOfVariables 
     * @param variableDimension 
     * @return VectorDynamic 
     */
    VectorDynamic GenerateInitial(DAG_Model &dagTasks,
                                  vector<LLint> &sizeOfVariables,
                                  int variableDimension, VectorDynamic initialUser = GenerateVectorDynamic(1))
    {
        VectorDynamic initialEstimate;
        if (initialUser.norm() != 0)
        {
            LLint size = GenerateInitial(dagTasks, sizeOfVariables, variableDimension).rows();
            initialEstimate = initialUser;
            if (initialUser.rows() != size)
            {
                CoutError("User input initial vector has wrong length!");
            }
            return initialUser;
        }

        InitializeMethod _initializeMethod = initializeMethod;

        switch (_initializeMethod)
        {
        case IndexMode:
            initialEstimate = GenerateInitialForDAG_IndexMode(dagTasks,
                                                              sizeOfVariables, variableDimension);
            break;
        case FixedRelativeStart:
            initialEstimate = GenerateInitialForDAG_RelativeStart(dagTasks,
                                                                  sizeOfVariables,
                                                                  variableDimension);
            break;
        case RM:
            initialEstimate = GenerateInitial_RM(dagTasks,
                                                 sizeOfVariables,
                                                 variableDimension);
            break;
        case RM_DAG:
            initialEstimate = GenerateInitialForDAG_RM_DAG(dagTasks,
                                                           sizeOfVariables,
                                                           variableDimension);
        default:
            initialEstimate = GenerateInitialForDAG_RM_DAG(dagTasks,
                                                           sizeOfVariables,
                                                           variableDimension);
        }
        return initialEstimate;
    }

}