#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "EliminationForest_utils.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    void AddDAG_Factor(NonlinearFactorGraph &graph, DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo)
    {

        LLint errorDimensionDBF = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            size_t idNextTask = itr->first;
            for (size_t i = 0; i < tasksPrev.size(); i++)
            {
                LLint indexNext = IndexTran_Instance2Overall(idNextTask, 0, tasksInfo.sizeOfVariables);
                Symbol keyNext = GenerateKey(idNextTask, indexNext);
                LLint indexPrev = IndexTran_Instance2Overall(tasksPrev[i].id, 0, tasksInfo.sizeOfVariables);
                Symbol keyPrev = GenerateKey(tasksPrev[i].id, indexPrev);
                double execTime = tasksPrev[i].executionTime;
                NormalErrorFunction2D DAG2D =
                    [execTime](VectorDynamic x1, VectorDynamic x2)
                {
                    VectorDynamic res = x1;
                    res(0, 0) = Barrier(x1(0, 0) - x2(0, 0) - execTime);
                    return res;
                };
                graph.emplace_shared<InequalityFactor2D>(keyNext, keyPrev, DAG2D, model);
            }
        }
    }

}