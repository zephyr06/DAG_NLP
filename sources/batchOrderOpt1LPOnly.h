#pragma once
#include <iostream>

#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "sources/Tools/VariadicTable.h"

#include "sources/Optimization/Optimize.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Baseline/OptimizeSA.h"
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Baseline/RTSS21IC.h"
// #include "sources/Optimization/OptimizeOrder.h"
#include "sources/Optimization/OptimizeSFOrder.h"
// #include "sources/batchOptimize.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Baseline/VerucchiRTDABridge.h"

using namespace std::chrono;

void BatchOptimizeOrder()
{
    std::string dirStr = PROJECT_PATH + "TaskData/dagTasks/";
    const char *pathDataset = (dirStr).c_str();
    // std::cout << "Dataset Directory: " << pathDataset << std::endl;
    std::vector<std::vector<double>> runTimeAll(TotalMethodUnderComparison);
    std::vector<std::vector<double>> objsAll(TotalMethodUnderComparison);
    std::vector<std::vector<int>> schedulableAll(TotalMethodUnderComparison); // values could only be 0 / 1

    std::vector<string> errorFiles;
    std::vector<string> worseFiles;
    std::vector<string> files = ReadFilesInDirectory(pathDataset);
    for (const auto &file : files)
    {
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag" && file.find("Res") == std::string::npos)
        {
            std::cout << file << endl;
            string path = PROJECT_PATH + "TaskData/dagTasks/" + file;
            OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(path, priorityMode);
            if (dagTasks.GetSfBound() > 0)
                sensorFusionTolerance = dagTasks.GetSfBound();
            if (dagTasks.GetRtdaBound() > 0)
                FreshTol = dagTasks.GetRtdaBound();

            // int N = dagTasks.tasks.size();
            AssertBool(true, dagTasks.chains_.size() > 0, __LINE__);
            for (int batchTestMethod = 0; batchTestMethod < TotalMethodUnderComparison; batchTestMethod++)
            {
                // only do initial and OrderOpt1LP, pass other methods
                if (batchTestMethod == 1 || batchTestMethod == 2 || batchTestMethod == 3 || batchTestMethod == 4)
                    continue;

                if (considerSensorFusion != 0 && batchTestMethod == 2)
                    continue;
                else if (considerSensorFusion == 0 && batchTestMethod == 3)
                    continue;
                double obj;
                int schedulable;
                OrderOptDAG_SPACE::ScheduleResult res;
                if (VerifyResFileExist(pathDataset, file, batchTestMethod))
                {
                    res = ReadFromResultFile(pathDataset, file, batchTestMethod);
                }
                else
                {
                    auto start = chrono::high_resolution_clock::now();
                    if (batchTestMethod == 0)
                    {
                        res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT(dagTasks, coreNumberAva, sensorFusionTolerance, FreshTol);
                    }
                    else if (batchTestMethod == 1)
                    {
                        doScheduleOptimization = 1;
                        doScheduleOptimizationOnlyOnce = 0;
                        res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel(dagTasks, coreNumberAva);
                    }
                    else if (batchTestMethod == 2)
                    {
                        res = ScheduleVerucchiRTDA(dagTasks, dagTasks.chains_, coreNumberAva, 15.0, 400000.0, 15.0, 400000.0, 15.0);
                    }
                    else if (batchTestMethod == 3)
                    {
                        res = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, sensorFusionTolerance, FreshTol);
                    }
                    else if (batchTestMethod == 4)
                    {
                        doScheduleOptimization = 0;
                        res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel(dagTasks, coreNumberAva);
                    }
                    else if (batchTestMethod == 5)
                    {
                        doScheduleOptimization = 1;
                        doScheduleOptimizationOnlyOnce = 1;
                        res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel(dagTasks, coreNumberAva);
                    }
                    else
                    {
                        CoutError("Please provide batchTestMethod implementation!");
                    }
                    auto stop = chrono::high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    res.timeTaken_ = double(duration.count()) / 1e6;
                    // res.rtda_.print();
                }
                std::cout << "Schedulable? " << res.schedulable_ << std::endl;
                std::cout << "Objective: " << res.obj_ << std::endl;

                if (res.schedulable_ == false && batchTestMethod != 0) // If optimized schedule is not schedulable, use list scheduling instead
                {
                    res.obj_ = objsAll[0].back();
                    errorFiles.push_back(file);
                }
                WriteToResultFile(pathDataset, file, res, batchTestMethod);

                runTimeAll[batchTestMethod].push_back(res.timeTaken_);
                schedulableAll[batchTestMethod].push_back((res.schedulable_ ? 1 : 0));
                objsAll[batchTestMethod].push_back(res.obj_);
            }
            // if (objsAll[2].size() > 0 && objsAll[1].back() > objsAll[2].back())
            // {
            //     CoutWarning("One case where proposed method performs worse is found: " + file);
            //     worseFiles.push_back(file);
            // }
        }
    }

    int n = objsAll[0].size();
    if (n != 0)
    {
        VariadicTable<std::string, double, double, double> vt({"Method", "Schedulable ratio", "Obj (Only used in RTDA experiment)", "TimeTaken"}, 10);

        vt.addRow("Initial", Average(schedulableAll[0]), Average(objsAll[0]), Average(runTimeAll[0]));
        // vt.addRow("TOM", Average(schedulableAll[1]), Average(objsAll[1]), Average(runTimeAll[1]));
        // vt.addRow("OrderOpt", Average(schedulableAll[4]), Average(objsAll[4]), Average(runTimeAll[4]));
        vt.addRow("OrderOpt1LP", Average(schedulableAll[5]), Average(objsAll[5]), Average(runTimeAll[5]));
        // vt.addRow("Verucchi20RTAS", Average(schedulableAll[2]), Average(objsAll[2]), Average(runTimeAll[2]));
        // vt.addRow("Wang21RTSS_IC", Average(schedulableAll[3]), Average(objsAll[3]), Average(runTimeAll[3]));
        // vt.addRow("Initial", Average(objsAll[0]), Average(runTimeAll[0]));

        vt.print(std::cout);
    }

    // std::cout << "The number of error files: " << errorFiles.size() << std::endl;
    // for (string file : errorFiles)
    //     std::cout << file << std::endl;

    // std::cout << "The number of files where OrderOpt performs worse: " << worseFiles.size() << std::endl;
    // for (string file : worseFiles)
    //     std::cout << file << std::endl;
    return;
}