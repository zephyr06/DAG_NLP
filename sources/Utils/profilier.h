#pragma once

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <vector>
#include <unordered_map>

#include <unistd.h> // only effective with Linux, "windows.h" works with Windows

#include "sources/Utils/colormod.h"
#include "sources/Utils/testMy.h"

#define CurrentTimeInProfiler std::chrono::high_resolution_clock::now()
#define BeginTimerAppInProfiler BeginTimer(__FUNCTION__);
#define EndTimerAppInProfiler EndTimer(__FUNCTION__);
extern std::mutex mtx_profiler;
struct ProfilerData
{
    typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimerType;
    TimerType begin;
    TimerType end;
    double accum;
    ProfilerData() : begin(std::chrono::high_resolution_clock::now()),
                     end(std::chrono::high_resolution_clock::now()), accum(0) {}
    void UpdateAccum()
    {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin);
        accum += double(duration.count()) / 1e6;
    }
};

static std::unordered_map<std::string, ProfilerData> profilerMap;

void BeginTimer(std::string funcName);

void EndTimer(std::string funcName, bool print = false);

struct TimerDataProfiler
{
    std::string name;
    double accum;
};
bool compareProfiler(TimerDataProfiler a, TimerDataProfiler b);
void PrintTimer();