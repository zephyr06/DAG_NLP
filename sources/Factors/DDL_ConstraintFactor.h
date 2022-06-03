#include "sources/Factors/BaseSchedulingFactor.h"

using namespace RegularTaskSystem;
using namespace DAG_SPACE;

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

void AddDDL_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo, bool ifPreemptive = false)
{

    auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma / weightDDL_factor);
    auto model2 = noiseModel::Isotropic::Sigma(2, noiseModelSigma / weightDDL_factor);
    TaskSet &tasks = tasksInfo.tasks;

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            Symbol key = GenerateKey(i, j);

            if (ifPreemptive)
            {
                graph.emplace_shared<DDLFactorPreempt>(key, j * tasks[i].period,
                                                       tasks[i].deadline + j * tasks[i].period, model2);
            }
            else
            {

                // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
                graph.emplace_shared<SmallerThanFactor1D>(key,
                                                          tasks[i].deadline - tasks[i].executionTime + (j)*tasks[i].period, model1);
                // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                graph.emplace_shared<LargerThanFactor1D>(key, j * tasks[i].period, model1);
            }
        }
    }
}
