#include "sources/OptimizeSA.h"

TEST(DAG_Generated, v1)
{
    using namespace RTSS21IC_NLP;
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/" + testDataSetName + ".csv", "orig");

    auto res = OptimizeSchedulingSA(tasks);
    cout << "The error after optimization is " << Color::green << res.initialError << Color::def << endl;
    cout << "The result after optimization is " << Color::blue << res.optimizeVariable << Color::def << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
