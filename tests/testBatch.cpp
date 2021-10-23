#include "../sources/batchOptimize.h"
TEST(parameters, a1)
{
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