#pragma once
#include "sources/Utils/DeclareDAG.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Factors/DBF_utils.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/InequalifyFactor.h"

class BaseSchedulingFactor : public NoiseModelFactor1<VectorDynamic>
{
public:
    TaskSetInfoDerived tasksInfo;
    // TaskSet tasks;
    // vector<LLint> sizeOfVariables;
    // int N;
    LLint errorDimension;
    // LLint length;
    // each element contains tasks belonging to the same processor
    // ProcessorTaskSet processorTaskSet;
    EliminationForest forestInfo;
    // std::unordered_map<LLint, LLint> mapIndex_True2Compress;

    BaseSchedulingFactor(Key key, TaskSetInfoDerived &tasksInfo,
                         EliminationForest &forestInfo, LLint errorDimension,
                         SharedNoiseModel model)
        : NoiseModelFactor1<VectorDynamic>(model, key), tasksInfo(tasksInfo),
          //   tasks(tasksInfo.tasks), sizeOfVariables(tasksInfo.sizeOfVariables),
          //   N(tasks.size()),
          errorDimension(errorDimension),
          //   processorTaskSet(tasksInfo.processorTaskSet),
          forestInfo(forestInfo)
    {
        // length = tasksInfo.length;
    }

    virtual Vector evaluateError(const VectorDynamic &startTimeVector,
                                 boost::optional<Matrix &> H = boost::none) const override
    {
        CoutError("evaluateError must be implemented in derived class!");
        VectorDynamic a;
        return a;
    }
};