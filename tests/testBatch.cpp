#include "sources/batchOptimizeOrder.h"
TEST(parameters, a1)
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