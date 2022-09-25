#pragma once

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"

namespace DAG_SPACE
{

    void PrintKeyVector(gtsam::KeyVector &vec)
    {
        for (uint i = 0; i < vec.size(); i++)
        {
            gtsam::Symbol{vec[i]}.print();
            std::cout << ", ";
        }
        std::cout << endl;
    }

    std::vector<gtsam::KeyVector> FindJobIndexWithError(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo, NonlinearFactorGraph &graph)
    {

        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        std::vector<gtsam::KeyVector> indexPairsWithError;
        // go through each factor
        for (auto itr = graph.begin(); itr != graph.end(); itr++)
        {
            if ((*itr)->error(initialEstimateFG) != 0)
            {
                gtsam::KeyVector keys = (*itr)->keys();
                indexPairsWithError.push_back(keys);
                itr->get()->printKeys();
                std::cout << (*itr)->error(initialEstimateFG) << std::endl;
            }
        }
        return indexPairsWithError;
    }

    VectorDynamic JobGroupsOptimize(VectorDynamic &initialEstimate, TaskSetInfoDerived &tasksInfo)
    {
        return initialEstimate;
        // std::vector<LLint> FindJobIndexWithError(...) e.g., {3,1,4,8}
        // std::vector<std::vector<LLint>> GroupIndexWithError(...) e.g., {{3,1,4,8}}
        // VectorDynamic AdjustIndexOrderWithinEachGroup(std::vector<std::vector<LLint>> groupIndex, VectorDynamic initialEstimate)
    }

} // namespace DAG_SPACE