#include "../sources/OptimizeSA.h"

TEST(DAG_Generated, v1)
{
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/" + testDataSetName + ".csv", "orig");

    VectorDynamic res = OptimizeSchedulingSA(tasks);
    cout << "The result after optimization is " << Color::blue << res << Color::def << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
