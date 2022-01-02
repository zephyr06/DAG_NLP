#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"
#include "../sources/MakeSpanFactor.h"
#include "../sources/MultiKeyFactor.h"
/**
 * @brief 
 * 
 * @param tasksInfo 
 * @param mapPrev 
 * @param type 's' means start, 'e' means end, other types are not reconized!
 * @return vector<gtsam::Symbol> 
 */
std::vector<gtsam::Symbol> GenerateKeysMS(TaskSetInfoDerived &tasksInfo,
                                          MAP_Prev &mapPrev, char type)
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

void AddMakeSpanFactor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo, MAP_Prev &mapPrev)
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
TEST(GenerateKeysMS, start)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto keyS = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 's');
    auto keyE = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 'e');
    vector<int> b = {2, 3, 4};

    vector<LLint> c = {0, 0, 0};
    std::vector<gtsam::Symbol> exp1 = GenerateKey(b, c);
    AssertBool(true, exp1 == keyS, __LINE__);
    vector<int> a = {0};
    vector<LLint> d = {1};
    std::vector<gtsam::Symbol> exp2 = GenerateKey(a, d);
    AssertBool(true, exp2 == keyE, __LINE__);
}
TEST(GenerateKeysMS, start2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v3.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto keyS = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 's');
    auto keyE = GenerateKeysMS(tasksInfo, dagTasks.mapPrev, 'e');
    vector<int> b = {0, 1, 2, 3, 4};

    vector<LLint> c = {0, 0, 0, 0, 0};
    std::vector<gtsam::Symbol> exp1 = GenerateKey(b, c);
    AssertBool(true, exp1 == keyS, __LINE__);
    vector<int> a = {0, 1, 2, 3, 4};
    vector<LLint> d = {3, 1, 1, 0, 0};
    std::vector<gtsam::Symbol> exp2 = GenerateKey(a, d);
    AssertBool(true, exp2 == keyE, __LINE__);
}
TEST(TESTms, v1)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 6, 107, 5, 3, 104, 2, 0, 101;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    // initialEstimateFG.print();
    double actual = graph.error(initialEstimateFG);
    double expect = 6844.5;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}
TEST(TESTms, v2)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    makespanWeight = 1;
    NonlinearFactorGraph graph;
    AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);

    VectorDynamic startTimeVector;
    startTimeVector.resize(8, 1);
    startTimeVector << 1, 2, 3, 4, 5, 6, 7, 8;
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    // initialEstimateFG.print();
    double actual = graph.error(initialEstimateFG);
    double expect = 32;
    AssertEqualScalar(expect, actual, 1e-6, __LINE__);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}