#include "../sources/Optimize.h"

TEST(ExtractVariable, v1)
{
    using namespace DAG_SPACE;
    auto res = OptimizeTaskSystem1();
    cout << "The result after optimization is " << res << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}