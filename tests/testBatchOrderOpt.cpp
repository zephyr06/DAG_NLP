#include "sources/batchOrderOptOnly.h"
TEST(batchVerucchi, verucchi)
{
    // BeginTimer("main");
    BatchOptimizeOrder();
    // EndTimer("main");
    // PrintTimer();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}