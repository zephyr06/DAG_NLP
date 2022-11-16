#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/OptimizeOrderUtils.h"
// #include "sources/Factors/RTDA_Factor.h"
// #include "sources/Factors/Interval.h"
// #include "sources/Optimization/TopologicalSort.h"
// #include "sources/Baseline/VerucchiRTDABridge.h"
// #include "sources/Utils/BatchUtils.h"
// #include "sources/batchOptimizeSFOrder.h"

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
