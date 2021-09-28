#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/GraphUtilsFromBGL.h"

TEST(DAG_Optimize_schedule, v1)
{
    using namespace DAG_SPACE;
    DAG_Model tasks = ReadDAG_Tasks("../TaskData/" + testDataSetName + ".csv", "orig");

    auto sth = OptimizeScheduling(tasks);

    VectorDynamic res = sth.optimizeVariable;

    cout << "The result after optimization is " << Color::green << sth.optimizeError
         << Color::blue << res << Color::def << endl;
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}