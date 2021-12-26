#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

using namespace DAG_SPACE;

// class DBF_Factor : public InequalityFactor2D
// {
// public:
//     TaskSetInfoDerived tasksInfo;
//     LLint index1;
//     LLint index2;

// };

void AddDBF_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
{

    LLint errorDimensionDBF = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    TaskSet &tasks = tasksInfo.tasks;

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            Symbol key = GenerateKey(index_overall);

            for (int ii = i + 1; ii < tasksInfo.N; ii++)
            {
                for (int jj = 0; jj < int(tasksInfo.sizeOfVariables[ii]); jj++)
                {
                    LLint index_overall_inner = IndexTran_Instance2Overall(ii, jj, tasksInfo.sizeOfVariables);
                    Symbol key_inner = GenerateKey(index_overall_inner);
                    NormalErrorFunction2D DBF2D =
                        [tasksInfo, index_overall, index_overall_inner](VectorDynamic x1, VectorDynamic x2)
                    {
                        Interval v1 = CreateSingleInterval(index_overall, x1(0, 0),
                                                           tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        Interval v2 = CreateSingleInterval(index_overall_inner, x2(0, 0),
                                                           tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        VectorDynamic res = x1;
                        res << Overlap(v1, v2);
                        return res;
                    };
                    // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
                    graph.emplace_shared<InequalityFactor2D>(key, key_inner, 1, DBF2D, model);
                }
            }
            // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
            graph.emplace_shared<SmallerThanFactor1D>(key, tasks[i].deadline + j * tasks[i].period, weightDDL_factor, model);
        }
    }
}

TEST(testDDL, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddDBF_Factor(graph, tasksInfo);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    double expect = 641;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}