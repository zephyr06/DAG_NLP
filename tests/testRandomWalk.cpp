
#include "../sources/Optimize.h"
#include "../sources/testMy.h"

using namespace DAG_SPACE;
/**
 * @brief 
 * 
 * @param startTimeVector (tasksInfo.variableDimension, 1)
 * @param tasksInfo 
 * @param forestInfo 
 * @return VectorDynamic 
 */
VectorDynamic RandomWalk(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo,
                         EliminationForest &forestInfo)
{
    // vector<LLint> vanishGradientIndex =
    //     FindVanishIndex(startTimeVector, tasksInfo.tasks, tasksInfo.sizeOfVariables, forestInfo);
    // VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
    //                                                        forestInfo);

    vector<LLint> indexes;
    indexes.reserve(startTimeVector.size());
    for (uint i = 0; i < startTimeVector.size(); i++)
        indexes.push_back(i);
    vector<Interval> intervalVec =
        CreateIntervalFromSTVSameOrder(indexes,
                                       startTimeVector, tasksInfo.tasks, tasksInfo.sizeOfVariables);

    LLint variableDimension = intervalVec.size();
    vector<LLint> coverIntervalIndex;
    coverIntervalIndex.reserve(variableDimension);
    std::unordered_set<LLint> indexSetBig;

    VectorDynamic stvWalk = startTimeVector;
    for (LLint i = 0; i < variableDimension; i++)
    {
        double s1 = intervalVec[i].start;
        double f1 = s1 + intervalVec[i].length;
        for (LLint j = i + 1; j < variableDimension; j++)
        {
            double s2 = intervalVec[j].start;
            double f2 = s2 + intervalVec[j].length;

            if ((s2 > s1 && f2 < f1) || (s2 < s1 && f2 > f1))
            {
                LLint leafIndex = FindLeaf(i, forestInfo.mapIndex);
                if (indexSetBig.find(leafIndex) == indexSetBig.end())
                {
                    indexSetBig.insert(leafIndex);
                }
                leafIndex = FindLeaf(j, forestInfo.mapIndex);
                if (indexSetBig.find(leafIndex) == indexSetBig.end())
                {
                    indexSetBig.insert(leafIndex);
                }
            }
        }
    }

    // auto m = MapIndex_True2Compress(forestInfo.maskForEliminate);
    // vector<LLint> coverIndexInCompressed;
    // coverIndexInCompressed.reserve(startTimeVectorOrig.rows());
    // for (auto itr = indexSetBig.begin(); itr != indexSetBig.end(); itr++)
    // {
    //     coverIndexInCompressed.push_back(m[(*itr)]);
    // }
    // return coverIndexInCompressed;
    return startTimeVector;
}
TEST(a, b)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);

    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(tasksInfo);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;
    VectorDynamic initial = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables,
                                            tasksInfo.variableDimension);
    initial << 6, 0, 1, 2, 5;
    VectorDynamic actual = RandomWalk(initial, tasksInfo, forestInfo);
    VectorDynamic expect = initial;
    expect << 3, 0, 1, 2, 5;
    // assert_equal(expect, actual);
    // AssertEigenEqualVector(expect, actual, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}