#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/testMy.h"
#include "sources/Utils/Parameters.h"
// #include "sources/batchOptimizeSFOrder.h"
#include "sources/Factors/ObjectiveFunctions.h"
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
