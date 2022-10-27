#include "sources/batchWangNLPOnly.h"
TEST(batchWangNLP, verucchi)
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