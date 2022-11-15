
#include "sources/Optimization/ScheduleSimulation.h"
namespace OrderOptDAG_SPACE
{

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

    VectorDynamic SimulateFixedPrioritySched(const DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, LLint timeInitial)
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

    // TODO: when two jobs have same priority, choose the one with higher precedence priority
    VectorDynamic ListSchedulingLFTPA(DAG_Model &dagTasks,
                                      TaskSetInfoDerived &tasksInfo, int processorNum,
                                      boost::optional<std::vector<uint> &> processorIdVec)
    {
        // BeginTimerAppInProfiler;
        const TaskSet &tasks = dagTasks.tasks;
        VectorDynamic initial = GenerateVectorDynamic(tasksInfo.variableDimension);

        // contains the index of tasks to run
        RunQueue runQueue(tasks);

        vector<bool> busy(processorNum, false);
        vector<LLint> nextFree(processorNum, -1);

        // std::vector<LLint> jobScheduled(jobOrder ? ((*jobOrder).size()) : 0, -1); // order is the same as JobOrder's jobs
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

                    p = runQueue.popLeastFinishTime(tasksInfo);
                    findTaskToSchedule = true;

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
        // EndTimerAppInProfiler;
        return initial;
    }

    VectorDynamic SFOrderScheduling(DAG_Model &dagTasks,
                                    const TaskSetInfoDerived &tasksInfo, int processorNum, const SFOrder &jobOrder,
                                    boost::optional<std::vector<uint> &> processorIdVec)
    {
        BeginTimerAppInProfiler;
        const TaskSet &tasks = dagTasks.tasks;
        const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

        VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
        // return startTimeVector;
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
                    // TODO: jobOrder.size()/2
                    startTimeVector = GenerateVectorDynamic(tasksInfo.variableDimension);
                    startTimeVector(0) = -1;
                    break;
                }
                else
                {
                    timeNow = scheduledFinishTime[GetJobUniqueId(currentInstance.job, tasksInfo)];
                }
            }
            else if (currentInstance.type == 's')
            {
                bool currentJobScheduled = false;
                while (!currentJobScheduled)
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
                            currentJobScheduled = true;
                            break;
                        }
                    }
                    if (!currentJobScheduled)
                    { // cant't find available processor, wait to earliest available time
                        LLint leastAvailableProcessorTime = LLONG_MAX;
                        for (int processorId = 0; processorId < processorNum; processorId++)
                        {
                            if (leastAvailableProcessorTime > nextFree[processorId])
                            {
                                leastAvailableProcessorTime = nextFree[processorId];
                            }
                        }
                        timeNow = leastAvailableProcessorTime;
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
                startTimeVector(0) = -1;
                break;
            }
        }

        EndTimerAppInProfiler;
        return startTimeVector;
    }
} // namespace OrderOptDAG_SPACE