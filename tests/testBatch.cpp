#include <CppUnitLite/TestHarness.h>
#include "sources/batchOptimizeSFOrder.h"
TEST(all, a1)
{
    BeginTimer("main");
    BatchOptimizeOrder();
    EndTimer("main");
    PrintTimer();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}