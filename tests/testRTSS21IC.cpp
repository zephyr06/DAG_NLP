#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Baseline/RTSS21IC/sources/Optimize.h"
#include "sources/Baseline/RTSS21IC/sources/GraphUtilsFromBGL.h"

TEST(DAG_Optimize_schedule, v1)
{
    RTSS21IC_NLP::DAG_SPACE::DAG_Model tasks = RTSS21IC_NLP::DAG_SPACE::ReadDAG_Tasks(RTSS21IC_NLP::PROJECT_PATH + "TaskData/" + RTSS21IC_NLP::testDataSetName + ".csv", "orig");
    auto sth = OptimizeScheduling(tasks);

    RTSS21IC_NLP::VectorDynamic res = sth.optimizeVariable;

    std::cout << "The result after optimization is " << RTSS21IC_NLP::Color::green << sth.optimizeError
              //  << Color::blue << res
              << RTSS21IC_NLP::Color::def << std::endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
