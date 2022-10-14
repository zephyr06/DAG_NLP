#include "sources/batchOptimize.h"
TEST(parameters, a1)
{
    using namespace RTSS21IC_NLP;
    BeginTimer("main");
    BatchOptimize();
    EndTimer("main");
    PrintTimer();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}