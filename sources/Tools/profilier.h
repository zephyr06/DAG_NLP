#pragma once

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <vector>
#include <unordered_map>

#include <unistd.h> // only effective with Linux, "windows.h" works with Windows

#include "sources/Tools/colormod.h"
#include "sources/Tools/testMy.h"

#define CurrentTimeInProfiler std::chrono::high_resolution_clock::now()
#define BeginTimerAppInProfiler BeginTimer(__FUNCTION__);
#define EndTimerAppInProfiler EndTimer(__FUNCTION__);
std::mutex mtx_profiler;
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

std::unordered_map<std::string, ProfilerData> profilerMap;
int AAA;

void BeginTimer(std::string funcName)
{
    std::lock_guard<std::mutex> lock(mtx_profiler);
    auto itr = profilerMap.find(funcName);
    if (itr == profilerMap.end())
    {
        profilerMap[funcName] = ProfilerData();
    }
    else
    {
        profilerMap[funcName].begin = CurrentTimeInProfiler;
    }
}

void EndTimer(std::string funcName, bool print = false)
{
    std::lock_guard<std::mutex> lock(mtx_profiler);
    auto itr = profilerMap.find(funcName);
    if (itr == profilerMap.end())
    {
        CoutError("Timer cannot find entry!");
    }
    profilerMap[funcName].end = CurrentTimeInProfiler;
    profilerMap[funcName].UpdateAccum();
    if (print)
    {
        std::cout << "Total time spent by function " << funcName
                  << " is " << profilerMap[funcName].accum << " seconds" << std::endl;
    }
}

struct TimerDataProfiler
{
    std::string name;
    double accum;
};
bool compareProfiler(TimerDataProfiler a, TimerDataProfiler b)
{
    return a.accum > b.accum;
}
void PrintTimer()
{
    using namespace std;
    std::cout.precision(4);
    vector<TimerDataProfiler> vec;
    double totalProfile = 0;
    std::cout << Color::green << "Total time spent is: " << profilerMap["main"].accum << " seconds" << Color::def << std::endl;
    for (auto itr = profilerMap.begin(); itr != profilerMap.end(); itr++)
    {
        double perc = itr->second.accum / (profilerMap["main"].accum);
        vec.push_back(TimerDataProfiler{itr->first, perc});
    }
    sort(vec.begin(), vec.end(), compareProfiler);
    for (size_t i = 0; i < vec.size(); i++)
    {
        std::cout << Color::green << "Percentage: " << std::setfill('0') << std::setw(4) << vec[i].accum * 100.0
                  << "% Function name: " << vec[i].name << Color::def << std::endl;
        totalProfile += vec[i].accum;
    }
    std::cout << Color::green << "Total profiled portion: " << totalProfile - 1 << Color::def << std::endl;
}