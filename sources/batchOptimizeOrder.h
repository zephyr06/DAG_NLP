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
#include "sources/Optimization/OptimizeOrder.h"
#include "sources/batchOptimize.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
using namespace std::chrono;

void BatchOptimizeOrder()
{
    std::string dirStr = PROJECT_PATH + "TaskData/dagTasks/";
    const char *pathDataset = (dirStr).c_str();
    std::cout << "Dataset Directory: " << pathDataset << std::endl;
    std::vector<std::vector<double>> runTimeAll(4);
    std::vector<std::vector<double>> objsAll(4);

    std::vector<string> errorFiles;
    std::vector<string> files = ReadFilesInDirectory(pathDataset);
    for (const auto &file : files)
    {
        std::cout << file << endl;
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag" && file.find("Res") == std::string::npos)
        {
            string path = PROJECT_PATH + "TaskData/dagTasks/" + file;
            DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(path, priorityMode);
            // int N = dagTasks.tasks.size();
            AssertBool(true, dagTasks.chains_.size() > 0, __LINE__);

            for (int batchTestMethod = 0; batchTestMethod < 3; batchTestMethod++)
            {
                double obj, timeTaken;
                if (VerifyResFileExist(pathDataset, file, batchTestMethod))
                {
                    std::tie(obj, timeTaken) = ReadFromResultFile(pathDataset, file, batchTestMethod);
                }
                else
                {
                    auto start = chrono::high_resolution_clock::now();
                    DAG_SPACE::ScheduleResult res;
                    if (batchTestMethod == 0)
                    {
                        res = DAG_SPACE::ScheduleDAGLS_LFT(dagTasks);
                        res.rtda_.print();
                    }
                    else if (batchTestMethod == 1)
                    {
                        if (processorAssignmentMode == 0)
                            res = DAG_SPACE::ScheduleDAGModel<LSchedulingKnownTA>(dagTasks);
                        else if (processorAssignmentMode == 1)
                            res = DAG_SPACE::ScheduleDAGModel<LSchedulingFreeTA>(dagTasks);
                        res.rtda_.print();
                    }
                    else if (batchTestMethod == 2)
                    {
                        res = ScheduleVerucchiRTDA(dagTasks, dagTasks.chains_, 1, 15.0, 400000.0, 15.0, 400000.0, 15.0);
                        res.rtda_.print();
                    }
                    else
                    {
                        CoutError("Please provide batchTestMethod implementation!");
                    }
                    obj = res.obj_;

                    auto stop = chrono::high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    timeTaken = double(duration.count()) / 1e6;
                    if (!res.schedulable_)
                    {
                        errorFiles.push_back(file);
                    }
                    WriteToResultFile(pathDataset, file, obj, timeTaken, batchTestMethod);
                }

                objsAll[batchTestMethod].push_back(obj);
                runTimeAll[batchTestMethod].push_back(timeTaken);
            }
            if (objsAll[1].back() > objsAll[2].back())
            {
                CoutWarning("One case where proposed method performs worse is found: " + file);
                errorFiles.push_back(file);
            }
        }
    }

    int n = objsAll[0].size();
    if (n != 0)
    {
        VariadicTable<std::string, double, double> vt({"Method", "Obj", "TimeTaken"}, 10);

        vt.addRow("Initial", Average(objsAll[0]), Average(runTimeAll[0]));
        vt.addRow("OrderOpt", Average(objsAll[1]), Average(runTimeAll[1]));
        vt.addRow("Verucchi20", Average(objsAll[2]), Average(runTimeAll[2]));
        // vt.addRow("Initial", Average(objsAll[0]), Average(runTimeAll[0]));

        vt.print(std::cout);
    }

    std::cout << "The number of error files: " << errorFiles.size() << std::endl;
    for (string file : errorFiles)
        std::cout << file << std::endl;
    // std::cout << Color::green << "Average error after optimization (accepted) is " << avEnergy << Color::def << endl;
    // std::cout << Color::green << "Average time consumed (accepted) is " << aveTime << Color::def << endl;
    // std::cout << Color::green << "The number of tasksets under analyzation is " << averageErrorAccept.size() << Color::def << endl;
    // std::cout << Color::green << "Total test cases: " << TotalTestCases << Color::def << endl;
    // std::cout << Color::blue << "Accept rate (Tol=1.0)" << double(averageErrorAccept1.size()) / TotalTestCases << Color::def << endl;
    // std::cout << Color::blue << "Accept rate (Tol=0.1)" << double(averageErrorAccept2.size()) / TotalTestCases << Color::def << endl;
    // std::cout << endl;

    // std::cout << Color::blue << "Accept rate " << double(averageErrorAccept.size()) / TotalTestCases << Color::def << endl;
    // std::cout << Color::blue << "Average initial error (all tasks) is " << avInitialError << Color::def << endl;
    // std::cout << Color::blue << "Average error after optimization (all tasks) is " << averrorAfterOpt << Color::def << endl;
    // std::cout << Color::blue << "Average time consumed (all) is " << avTimeall << Color::def << endl;

    // std::cout << Color::def;
    return;
}