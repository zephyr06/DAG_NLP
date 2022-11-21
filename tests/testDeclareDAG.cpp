
#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Utils/testMy.h"
using namespace GlobalVariablesDAGOpt;
TEST(ExtractVariable, v1)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v17.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
   std::vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104, 2, 0, 102;
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 0, 0));
    AssertEqualScalar(107, ExtractVariable(initialSTV, sizeOfVariables, 0, 1));
    AssertEqualScalar(5, ExtractVariable(initialSTV, sizeOfVariables, 1, 0));
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 2, 0));
    AssertEqualScalar(104, ExtractVariable(initialSTV, sizeOfVariables, 2, 1));
    AssertEqualScalar(2, ExtractVariable(initialSTV, sizeOfVariables, 3, 0));
    AssertEqualScalar(0, ExtractVariable(initialSTV, sizeOfVariables, 4, 0));
    AssertEqualScalar(102, ExtractVariable(initialSTV, sizeOfVariables, 4, 1));
}

TEST(ExtractVariable, v2)
{
    using namespace OrderOptDAG_SPACE;
    OrderOptDAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v20.csv", "orig");
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
   std::vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }
    auto initialSTV = GenerateInitialForDAG_IndexMode(dagTasks, sizeOfVariables, variableDimension);
    initialSTV << 3, 107, 5, 3, 104;
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 0, 0));
    AssertEqualScalar(107, ExtractVariable(initialSTV, sizeOfVariables, 1, 0));
    AssertEqualScalar(5, ExtractVariable(initialSTV, sizeOfVariables, 2, 0));
    AssertEqualScalar(3, ExtractVariable(initialSTV, sizeOfVariables, 3, 0));
    AssertEqualScalar(104, ExtractVariable(initialSTV, sizeOfVariables, 4, 0));
}

TEST(BigIndex2TaskIndex, v1)
{
    using namespace OrderOptDAG_SPACE;
    TaskSet tasks = ReadTaskSet(PROJECT_PATH + "TaskData/test_n5_v3.csv", "orig");
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
   std::vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }

    if (0 != BigIndex2TaskIndex(0, sizeOfVariables) ||
        1 != BigIndex2TaskIndex(5, sizeOfVariables) ||
        2 != BigIndex2TaskIndex(6, sizeOfVariables) ||
        3 != BigIndex2TaskIndex(8, sizeOfVariables) ||
        4 != BigIndex2TaskIndex(9, sizeOfVariables))
    {
       std::cout << "Error in BigIndex2TaskIndex" << std::endl;
        throw;
    }
}

TEST(AnalyzeKey, v1)
{
    auto key1 = GenerateKey(0, 0);
    auto p1 = AnalyzeKey(key1);
    AssertEqualScalar(0, p1.first);
    AssertEqualScalar(0, p1.second);
}
TEST(AnalyzeKey, v2)
{
    auto key1 = GenerateKey(9, 9);
    auto p1 = AnalyzeKey(key1);
    AssertEqualScalar(9, p1.first);
    AssertEqualScalar(9, p1.second);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}