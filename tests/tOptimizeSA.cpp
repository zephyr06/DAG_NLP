#include "sources/Baseline/OptimizeSA.h"

TEST(DAG_Generated, v1)
{
    using namespace OrderOptDAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    auto res = OptimizeSchedulingSA(tasks);
    cout << "The error after optimization is " << Color::green << res.initialError << Color::def << endl;
    cout << "The result after optimization is " << Color::blue << res.optimizeVariable << Color::def << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
