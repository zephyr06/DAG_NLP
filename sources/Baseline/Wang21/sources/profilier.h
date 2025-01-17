#pragma once

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <vector>
#include <unordered_map>
#include "colormod.h"
#include "testMy.h"

namespace RTSS21IC_NLP
{

// using namespace std::;
#define CurrentTime std::chrono::high_resolution_clock::now()
#define BeginTimerApp BeginTimer(__FUNCTION__);
#define EndTimerApp EndTimer(__FUNCTION__);
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

    void BeginTimer(std::string funcName)
    {
        auto itr = profilerMap.find(funcName);
        if (itr == profilerMap.end())
        {
            profilerMap[funcName] = ProfilerData();
        }
        else
        {
            profilerMap[funcName].begin = CurrentTime;
        }
    }

    void EndTimer(std::string funcName)
    {
        auto itr = profilerMap.find(funcName);
        if (itr == profilerMap.end())
        {
            CoutError("Timer cannot find entry!");
        }
        profilerMap[funcName].end = CurrentTime;
        profilerMap[funcName].UpdateAccum();
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
        std::cout.precision(4);
        std::vector<TimerDataProfiler> vec;
        double totalProfile = 0;
        std::cout << Color::green << "Total time spent by the main function is: " << profilerMap["main"].accum << Color::def << std::endl;
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
} // namespace RTSS21IC_NLP
