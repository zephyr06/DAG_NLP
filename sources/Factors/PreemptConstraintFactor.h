#include "sources/Factors/BaseSchedulingFactor.h"

using namespace RegularTaskSystem;
using namespace DAG_SPACE;

/**
 * @brief Preempt factor, guarantee f_i >= s_i + c_i
 *
 */
class PreemptConstraintFactor : public InequalityFactor1D
{
public:
    double executionTime;
    PreemptConstraintFactor(Key key, double executionTime,
                            SharedNoiseModel model) : InequalityFactor1D(key, model), executionTime(executionTime)
    {
        f = [executionTime](const VectorDynamic &x)
        {
            VectorDynamic res = GenerateVectorDynamic(1);
            res << Barrier(x(1) - x(0) - executionTime);
            return res;
        };
    }
};

void AddPreempt_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo, bool ifPreemptive = false)
{

    auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightDDL_factor);
    TaskSet &tasks = tasksInfo.tasks;

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            Symbol key = GenerateKey(i, j);
            if (ifPreemptive)
            {
                graph.emplace_shared<PreemptConstraintFactor>(key,
                                                              tasks[i].executionTime, model1);
            }
            else
            {
                return;
            }
        }
    }
}
