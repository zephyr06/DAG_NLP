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
            Symbol keyNext = GenerateKey(indexNext);
            LLint indexPrev = IndexTran_Instance2Overall(tasksPrev[i].id, 0, tasksInfo.sizeOfVariables);
            Symbol keyPrev = GenerateKey(indexPrev);
            double execTime = tasksPrev[i].executionTime;
            NormalErrorFunction2D DAG2D =
                [execTime](VectorDynamic x1, VectorDynamic x2)
            {
                VectorDynamic res = x1;
                res(0, 0) = Barrier(x1(0, 0) - x2(0, 0) - execTime);
                return res;
            };
            graph.emplace_shared<InequalityFactor2D>(keyNext, keyPrev, 1, DAG2D, model);
        }
    }
}

TEST(testDAG, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    NonlinearFactorGraph graph;
    AddDAG_Factor(graph, dagTasks, tasksInfo);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double actual = graph.error(initialEstimateFG);
    double expect = 263;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}