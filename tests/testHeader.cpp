#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Optimization/ScheduleSimulation.h"

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
