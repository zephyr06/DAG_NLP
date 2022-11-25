#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/testMy.h"

// #include "sources/batchOptimizeSFOrder.h"
#include "sources/Optimization/ObjectiveFunctions.h"
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
