#include "../sources/OptimizeSA.h"

TEST(DAG_Generated, v1)
{
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("../TaskData/" + testDataSetName + ".csv", "orig");

    VectorDynamic res = OptimizeSchedulingSA(tasks);
    cout << "The result after optimization is " << blue << res << def << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}