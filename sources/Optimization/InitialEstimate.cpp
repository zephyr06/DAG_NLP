#include "sources/Optimization/InitialEstimate.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{

    gtsam::Values GenerateInitialFG(const VectorDynamic &startTimeVector, const TaskSetInfoDerived &tasksInfo)
    {
        gtsam::Values initialEstimateFG;
        gtsam::Symbol key('a', 0); // just declare the variable

        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables.at(i)); j++)
            {
                // LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);

                gtsam::Symbol key = GenerateKey(i, j);
                VectorDynamic v = GenerateVectorDynamic(1);
                v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);

                initialEstimateFG.insert(key, v);
            }
        }
        return initialEstimateFG;
    }

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
        vector<int> order = FindDependencyOrderDFS(dagTasks);
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
        // LLint currTime = 0;

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
        vector<int> order = FindDependencyOrderDFS(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);

        // LLint index = 0;
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
                                               int variableDimension, int topoSortMethod)
    {
        int N = dagTasks.tasks.size();
        TaskSet tasks = dagTasks.GetTasks();
        // vector<int> order = FindDependencyOrderDFS(dagTasks);
        std::vector<int> order = TopologicalSortMulti(dagTasks)[topoSortMethod];
        LLint hyperPeriod = HyperPeriod(tasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);

        // LLint index = 0;
        LLint currTime = 0;

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
            if (currTime >= hyperPeriod)
                continue;
            if (currTime > tasks[i].period)
            {
                for (int j = 1; j <= currTime / tasks[i].period; j++)
                {
                    LLint index_overall = IndexTran_Instance2Overall(i, j, sizeOfVariables);
                    initial(index_overall, 0) = (j)*tasks[i].period;
                }
            }
        }

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(tasks);
        int processorNum = processorTaskSet.size();
        // it maps from tasks[i].processorId to index in runQueues&busy&nextFree

        ProcessorId2Index processorId2Index = CreateProcessorId2Index(tasks);

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
                                  int variableDimension, VectorDynamic initialUser)
    {
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        VectorDynamic initialEstimate;
        // initialEstimate = GenerateVectorDynamic(5);
        // initialEstimate << 62, 70, 85, 88, 90;
        // return initialEstimate;
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
            initialEstimate = SimulateFixedPrioritySched(dagTasks,
                                                         tasksInfo);
            break;
        case RM_DAG:
            initialEstimate = GenerateInitialForDAG_RM_DAG(dagTasks,
                                                           sizeOfVariables,
                                                           variableDimension);
            break;
            // case Custom_DAG:
            //     initialEstimate = GenerateInitial_Custom_DAG(dagTasks,
            //                                                  sizeOfVariables,
            //                                                  variableDimension);
            //     break;

        case ListScheduling:
            initialEstimate = ListSchedulingLFTPA(dagTasks,
                                                  tasksInfo, coreNumberAva);
            break;
        default:
            initialEstimate = GenerateInitialForDAG_RM_DAG(dagTasks,
                                                           sizeOfVariables,
                                                           variableDimension);
        }
        return initialEstimate;
    }

}