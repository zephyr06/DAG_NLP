#include "../sources/Optimize.h"
TEST(random, v1)
{
    // VectorDynamic a;
    // a.resize(6, 1);
    // for (int i = 0; i < 6; i++)
    // {
    //     a(i, 0) = i;
    // }
    // cout << a << endl;
    ;
}

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