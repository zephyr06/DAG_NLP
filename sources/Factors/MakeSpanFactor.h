#pragma once

#include "sources/Utils/DeclareDAG.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/BaseSchedulingFactor.h"
#include "sources/Factors/MultiKeyFactor.h"

/**
 * @brief
 *
 * @param tasksInfo
 * @param mapPrev
 * @param type 's' means start, 'e' means end, other types are not reconized!
 * @return vector<gtsam::Symbol>
 */
std::vector<gtsam::Symbol> GenerateKeysMS(TaskSetInfoDerived &tasksInfo,
                                          DAG_SPACE::MAP_Prev &mapPrev, char type)
{
    std::unordered_set<int> keySetToExclude;
    for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
    {
        if (type == 's')
            keySetToExclude.insert(itr->first);
        else if (type == 'e')
        {
            for (auto itrr = itr->second.begin(); itrr != itr->second.end(); itrr++)
                keySetToExclude.insert(itrr->id);
        }
        else
        {
            CoutError("Input parameter type is not recognized in GenerateKeysMS!");
        }
    }
    std::vector<gtsam::Symbol> keyVec;
    keyVec.reserve(tasksInfo.N);
    for (int i = 0; i < tasksInfo.N; i++)
    {
        if (keySetToExclude.find(i) == keySetToExclude.end())
        {
            if (type == 's')
                keyVec.push_back(GenerateKey(i, 0));
            else if (type == 'e')
                keyVec.push_back(GenerateKey(i, tasksInfo.sizeOfVariables[i] - 1));
        }
    }
    return keyVec;
}

void AddMakeSpanFactor(NonlinearFactorGraph &graph,
                       TaskSetInfoDerived &tasksInfo, DAG_SPACE::MAP_Prev &mapPrev)
{
    LLint errorDimensionMS = 1;
    if (makespanWeight == 0)
        return;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma / makespanWeight);
    vector<gtsam::Symbol> keysBegin = GenerateKeysMS(tasksInfo, mapPrev, 's');
    uint beginSize = keysBegin.size();
    vector<gtsam::Symbol> keysAll = GenerateKeysMS(tasksInfo, mapPrev, 'e');
    keysAll.insert(keysAll.begin(), keysBegin.begin(), keysBegin.end());

    TaskSet &tasks = tasksInfo.tasks;
    LambdaMultiKey f = [beginSize, keysAll, tasks](const Values &x)
    {
        // const VectorDynamic &x0 = x.at<VectorDynamic>(keyVec[0]);
        // const VectorDynamic &x1 = x.at<VectorDynamic>(keyVec[1]);

        VectorDynamic res = GenerateVectorDynamic(1);
        double minStart = INT_MAX;
        double maxEnd = -1;
        for (uint i = 0; i < beginSize; i++)
        {
            minStart = min(minStart, x.at<VectorDynamic>(keysAll[i])(0, 0));
        }
        for (uint i = beginSize; i < keysAll.size(); i++)
        {
            auto p = AnalyzeKey(keysAll[i]);
            maxEnd = max(maxEnd, x.at<VectorDynamic>(keysAll[i])(0, 0) + tasks[p.first].executionTime);
        }
        res << maxEnd - minStart;
        return res;
    };
    graph.emplace_shared<MultiKeyFactor>(keysAll, f, model);
    return;
}
