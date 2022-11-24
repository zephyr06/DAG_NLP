#pragma once
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>
#include "assert.h"
#include "sources/Utils/VariadicTable.h"

// #include "sources/Optimization/Optimize.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Baseline/RTSS21IC.h"
#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Baseline/VerucchiRTDABridge.h"

using namespace std::chrono;
namespace OrderOptDAG_SPACE
{
    enum BaselineMethods
    {
        InitialMethod, // 0
        Verucchi20,    // 1
        Wang21,        // 2
        TOM,           // 3
        TOM_Fast,      // 4
        TOM_FastLP     // 5
    };

    OrderOptDAG_SPACE::ScheduleResult PerformSingleScheduling(DAG_Model &dagTasks, OptimizeSF::ScheduleOptions &scheduleOptions, BaselineMethods batchTestMethod);

    struct BatchResult
    {
        double schedulableRatio;
        double performance;
        double runTime;
    };

    void ClearResultFiles(std::string dataSetFolder = PROJECT_PATH + "TaskData/dagTasksPerfTest/");

    std::vector<BatchResult> BatchOptimizeOrder(std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods, std::string dataSetFolder = PROJECT_PATH + "TaskData/dagTasks/");

} // namespace OrderOptDAG_SPACE