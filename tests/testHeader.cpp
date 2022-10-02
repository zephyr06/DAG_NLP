#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Utils/Parameters.h"
#include "sources/Factors/DBF_ConstraintFactorPreemptive.h"
#include "sources/batchOptimizeOrder.h"

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
