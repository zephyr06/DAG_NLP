#pragma once

#include "sources/Factors/BaseSchedulingFactor.h"

using namespace RegularTaskSystem;
using namespace OrderOptDAG_SPACE;

/**
 * @brief DDL factor in preemptive case
 *
 */
class DDLFactorPreempt : public InequalityFactor1D
{
public:
    double periodBeginTime;
    double absoluteDeadline;
    DDLFactorPreempt(Key key, double periodBeginTime, double absoluteDeadline,
                     SharedNoiseModel model) : InequalityFactor1D(key, model),
                                               periodBeginTime(periodBeginTime),
                                               absoluteDeadline(absoluteDeadline)
    {
        f = [periodBeginTime, absoluteDeadline](const VectorDynamic &x)
        {
            VectorDynamic res = GenerateVectorDynamic(2);
            res << Barrier(x(0) - periodBeginTime),
                Barrier(absoluteDeadline - x(1));
            return res;
        };
    }
};

void AddDDL_JobFactor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo, int taskId, int jobId, SharedNoiseModel model1 = NULL)
{
    if (!NULL)
    {
        model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightDDL_factor);
    }
    Symbol key = GenerateKey(taskId, jobId);
    TaskSet &tasks = tasksInfo.tasks;
    // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
    graph.emplace_shared<SmallerThanFactor1D>(key,
                                              tasks[taskId].deadline - tasks[taskId].executionTime + (jobId)*tasks[taskId].period, model1);
    // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
    graph.emplace_shared<LargerThanFactor1D>(key, jobId * tasks[taskId].period, model1);
}

void AddDDL_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
{

    auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightDDL_factor);
    auto model2 = noiseModel::Isotropic::Sigma(2, noiseModelSigma / weightDDL_factor);

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            AddDDL_JobFactor(graph, tasksInfo, i, j, model1);
        }
    }
}
