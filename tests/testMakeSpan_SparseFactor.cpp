#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"
#include "../sources/MakeSpanFactor.h"

/**
 * @brief 
 * 
 * @param tasksInfo 
 * @param mapPrev 
 * @param type 's' means start, 'e' means end, other types are not reconized!
 * @return vector<gtsam::Symbol> 
 */
std::vector<gtsam::Symbol> GenerateStartKeys(TaskSetInfoDerived &tasksInfo,
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
            CoutError("Input parameter type is not recognized in GenerateStartKeys!");
        }
    }
    std::vector<gtsam::Symbol> keyVec;
    keyVec.reserve(tasksInfo.N);
    for (int i = 0; i < tasksInfo.N; i++)
    {
        if (keySetToExclude.find(i) == keySetToExclude.end())
        {
            keyVec.push_back(GenerateKey(i, 0));
        }
    }
    return keyVec;
}

// void AddMakeSpanFactor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo, MAP_Prev &mapPrev)
// {
//     LLint errorDimensionMS = 1;
//     if (makespanWeight == 0)
//         return;
//     auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma / makespanWeight);
//     vector<gtsam::Symbol> keysBegin = GenerateStartKeys(tasksInfo, mapPrev);
//     vector<gtsam::Symbol> keysEnd = GenerateEndKeys(tasksInfo, mapPrev);
//     vector<gtsam::Symbol> keysAll = CombineKeys();
//     LambdaMultiKey f = [bv, keyVec, dimension](const Values &x)
//     {
//         // const VectorDynamic &x0 = x.at<VectorDynamic>(keyVec[0]);
//         // const VectorDynamic &x1 = x.at<VectorDynamic>(keyVec[1]);

//         VectorDynamic res = GenerateVectorDynamic(dimension);
//         for (uint i = 0; i < dimension; i++)
//         {
//             res(i, 0) = bv[i] - x.at<VectorDynamic>(keyVec[i])(0, 0);
//         }
//         return res;
//     };
//     MultiKeyFactor factor(keyVec, f, model);
//     graph.emplace_shared<MultiKeyFactor>(keysAll, f, model);
//     return;
// }
TEST(GenerateStartKeys, start)
{
    using namespace DAG_SPACE;
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    auto keyS = GenerateStartKeys(tasksInfo, dagTasks.mapPrev, 's');
    auto keyE = GenerateStartKeys(tasksInfo, dagTasks.mapPrev, 'e');
    vector<int> b = {2, 3, 4};

    vector<LLint> c = {0, 0, 0};
    std::vector<gtsam::Symbol> exp1 = GenerateKey(b, c);
    AssertBool(true, exp1 == keyS, __LINE__);
    vector<int> a = {0};
    vector<LLint> d = {0};
    std::vector<gtsam::Symbol> exp2 = GenerateKey(a, d);
    AssertBool(true, exp2 == keyE, __LINE__);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}