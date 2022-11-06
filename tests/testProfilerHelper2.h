

#pragma once
#include "sources/Tools/profilier.h"
#include <iostream>
#include "tests/testProfilerHelper2.h"

void TestProfilerFunc4(double interval = 1)
{
    BeginTimerAppInProfiler;
    TestProfilerFunc3(interval);
    AAA = 4;
    std::cout << "TestProfilerFunc4: " << AAA << std::endl;
    EndTimerAppInProfiler;
}