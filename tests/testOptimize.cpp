#include "../sources/Optimize.h"

TEST(ExtractVariable, v1)
{
    auto res = OptimizeTaskSystem1();
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}