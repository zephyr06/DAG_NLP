#include <CppUnitLite/TestHarness.h>
#include "tests/testProfilerHelper.h"
#include "tests/testProfilerHelper2.h"
#include "sources/Tools/profilier.h"

void TestProfilerFunc1(double interval = 1)
{
    BeginTimerAppInProfiler;
    AAA = 1;
    std::cout << "TestProfilerFunc1: " << AAA << std::endl;
    sleep(interval);
    EndTimerAppInProfiler;
}
void TestProfilerFunc(double interval = 1)
{
    BeginTimerAppInProfiler;
    TestProfilerFunc1(interval);
    TestProfilerFunc1(interval);

    for (int i = 0; i < 3; i++)
        TestProfilerFunc3(interval);

    std::cout << "TestProfilerFunc: " << AAA << std::endl;
    TestProfilerFunc1(interval);
    for (int i = 0; i < 3; i++)
        TestProfilerFunc4(interval);
    std::cout << "TestProfilerFunc: " << AAA << std::endl;
    EndTimerAppInProfiler;
}

TEST(profiler, v1)
{
    BeginTimer("main");
    TestProfilerFunc(1);
    EndTimer("main");
    PrintTimer();
    // Expected print:
    // TestProfilerFunc1: 1
    // TestProfilerFunc1: 1
    // TestProfilerFunc4: 3
    // TestProfilerFunc4: 3
    // TestProfilerFunc4: 3
    // TestProfilerFunc: 3
    // TestProfilerFunc1: 1
    // TestProfilerFunc4: 3
    // TestProfilerFunc4: 4
    // TestProfilerFunc4: 3
    // TestProfilerFunc4: 4
    // TestProfilerFunc4: 3
    // TestProfilerFunc4: 4
    // TestProfilerFunc: 4
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
