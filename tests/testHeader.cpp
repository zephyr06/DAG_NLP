#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
// #include "sources/Utils/OptimizeOrderUtils.h"
// #include "sources/Baseline/VerucchiRTDABridge.h"
#include "sources/Utils/BatchUtils.h"

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
