#include "gtest/gtest.h"
#include "../sources/Optimize.h"
#include "../sources/testMy.h"

TEST(a, b)
{
    using namespace DAG_SPACE;
    using namespace RegularTaskSystem;

    DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v9.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
    vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {

        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }
    pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(dagTasks);
    Graph eliminationTrees = sth.first;
    indexVertexMap indexesBGL = sth.second;
    VectorDynamic initial = GenerateInitial(dagTasks, sizeOfVariables, variableDimension);
    initial << 6, 0, 1, 2, 5;
    VectorDynamic actual = RandomWalk(initial, dagTasks, eliminationTrees, indexesBGL);
    VectorDynamic expect = initial;
    expect << 3, 0, 1, 2, 5;
    assert_equal(expect, actual);
    AssertEigenEqualVector(expect, actual, __LINE__);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}