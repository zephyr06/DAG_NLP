/**
 * @file SensorFusionFactor.h
 * @author Sen Wang
 * @brief This file implements sensor fusion factor for non-preemptive case;
 * @version 0.1
 * @date 2022-06-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "sources/Utils/DeclareDAG.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Utils/JobCEC.h"

namespace OrderOptDAG_SPACE
{
    using namespace RegularTaskSystem;
    LLint CountSFError(const DAG_Model &dagTasks, const vector<LLint> &sizeOfVariables);

    struct IndexData
    {
        LLint index;
        double time;
    };

    pair<int, int> ExtractMaxDistance(vector<IndexData> &sourceFinishTime);

    inline double ExtractMaxDistance(vector<double> &sourceFinishTime)
    {
        return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
               *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
    }

    VectorDynamic ObtainSensorFusionError(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector);

    std::unordered_map<JobCEC, std::vector<JobCEC>> GetSensorMapFromSingleJob(
        const TaskSetInfoDerived &tasksInfo, int task_id, TaskSet &precede_tasks, const VectorDynamic &x);

}