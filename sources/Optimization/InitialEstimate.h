
#pragma once
#include "unordered_map"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/TopologicalSort.h"
#include "sources/Utils/colormod.h"
#include "sources/Optimization/ScheduleSimulation.h"

using namespace RegularTaskSystem;
namespace OrderOptDAG_SPACE
{

    gtsam::Values GenerateInitialFG(const VectorDynamic &startTimeVector, const TaskSetInfoDerived &tasksInfo);

    /**
     * @brief Generate initial solution for the whole optimization
     *
     * @param tasks
     * @param sizeOfVariables
     * @return VectorDynamic size (N+1), first N is start time for nodes, the last one is r.h.s.
     */
    VectorDynamic GenerateInitialForDAG_IndexMode(DAG_Model &dagTasks,std::vector<LLint> &sizeOfVariables, int variableDimension);

    VectorDynamic GenerateInitialForDAG_RelativeStart(DAG_Model &dagTasks,
                                                     std::vector<LLint> &sizeOfVariables,
                                                      int variableDimension);

    // This method is not suitable to compare with Verucchi20 because it focus more on dependency, but less on schedulability (FT<Deadline), and RTDA.
    // VectorDynamic GenerateInitial_Custom_DAG(DAG_Model &dagTasks,
    //                                         std::vector<LLint> &sizeOfVariables,
    //                                          int variableDimension, LLint currTime = 0)
    // {
    //     priorityMode = "Assigned";
    //     // Assign priority for the task sets
    //     int N = dagTasks.tasks.size();
    //     TaskSet &tasks = dagTasks.tasks;
    //     TaskSetInfoDerived tasksInfo(tasks);
    //     //std::vector<int> order = FindDependencyOrderDFS(dagTasks);
    //     std::vector<int> order = TopologicalSortMulti(dagTasks)[3]; //"DM"
    //     for (int i = 0; i < N; i++)
    //     {
    //         tasks[i].priority_ = N - order[i];
    //     }

    //     return SimulateFixedPrioritySched(dagTasks, tasksInfo, 0);
    // }
    /**
     * @brief Warning! All the tasks's processorId must begin with 0, otherwise it reports Segmentation error.
     *
     * @param dagTasks
     * @param sizeOfVariables
     * @param variableDimension
     * @return VectorDynamic
     */
    VectorDynamic GenerateInitialForDAG_RM_DAG(DAG_Model &dagTasks,
                                              std::vector<LLint> &sizeOfVariables,
                                               int variableDimension, int topoSortMethod = 4);
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
                                 std::vector<LLint> &sizeOfVariables,
                                  int variableDimension, VectorDynamic initialUser = GenerateVectorDynamic(1));

}