

#include "sources/Tools/profilier.h"
#pragma once
void TestProfilerFunc3(double interval = 1)
{
    BeginTimerAppInProfiler;
    sleep(interval);
    AAA = 3;
    std::cout << "TestProfilerFunc4: " << AAA << std::endl;
    EndTimerAppInProfiler;
}