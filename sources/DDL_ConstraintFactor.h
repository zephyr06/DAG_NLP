#include "BaseSchedulingFactor.h"
namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    void AddDDL_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
    {

        LLint errorDimensionDDL = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma / weightDDL_factor);
        TaskSet &tasks = tasksInfo.tasks;

        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
            {
                LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
                Symbol key = GenerateKey(i, index_overall);

                // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
                graph.emplace_shared<SmallerThanFactor1D>(key,
                                                          tasks[i].deadline - tasks[i].executionTime + (j)*tasks[i].period, model);
                // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                graph.emplace_shared<LargerThanFactor1D>(key, j * tasks[i].period, model);
            }
        }
    }

}