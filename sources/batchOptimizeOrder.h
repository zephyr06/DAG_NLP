#pragma once
#include <iostream>

#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "sources/Optimization/Optimize.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Baseline/OptimizeSA.h"
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Optimization/OptimizeOrder.h"
#include "sources/batchOptimize.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
using namespace std::chrono;

void BatchOptimizeOrder()
{
    std::string dirStr = PROJECT_PATH + "TaskData/dagTasks";
    const char *pathDataset = (dirStr).c_str();
    std::cout << "Dataset Directory: " << pathDataset << std::endl;
    vector<double> runTimeAll;
    // int N;

    vector<string> errorFiles;
    ofstream outfileWrite;
    outfileWrite.open(PROJECT_PATH + "CompareWithBaseline/data_buffer_energy_task_number.txt",
                      std::ios_base::app);
    vector<string> files = ReadFilesInDirectory(pathDataset);
    int TotalTestCases = files.size() - 2;
    for (const auto &file : files)
    {
        std::cout << file << endl;
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag")
        {
            string path = PROJECT_PATH + "TaskData/dagTasks/" + file;
            DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(path, priorityMode);
            // int N = dagTasks.tasks.size();
            AssertBool(true, dagTasks.chains_.size() > 0, __LINE__);
            auto start = chrono::high_resolution_clock::now();
            DAG_SPACE::ScheduleResult res;
            if (batchTestMethod == 0)
            {
                res = DAG_SPACE::ScheduleDAGLS_LFT(dagTasks);
                res.rtda_.print();
            }
            else if (batchTestMethod == 1)
            {
                res = DAG_SPACE::ScheduleDAGModel(dagTasks);
                res.rtda_.print();
            }
            else if (batchTestMethod == 2)
            {
                DAG_SPACE::RTDA rtda = GetVerucchiRTDA(dagTasks, dagTasks.chains_, 1, 15.0, 400000.0, 15.0, 400000.0, 15.0);
                std::cout << Color::red;
                rtda.print();
                std::cout << Color::def;
            }

            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
            runTimeAll.push_back(timeTaken);

            if (!res.schedulable_ == -1)
            {
                errorFiles.push_back(file);
            }
        }
    }

    // double avEnergy = -1;
    // double aveTime = -1;
    // double averrorAfterOpt = -1;
    // double avInitialError = Average(errorInitial);
    // double avTimeall = Average(runTimeAll);
    // int n = runTimeAccept.size();
    // if (n != 0)
    // {
    //     avEnergy = Average(averageErrorAccept);
    //     aveTime = Average(runTimeAccept);
    //     averrorAfterOpt = Average(errorAfterOpt);
    // }

    // ofstream outfile1, outfile2;
    // outfile1.open(PROJECT_PATH + "CompareWithBaseline/ResultFiles/utilization.txt", std::ios_base::app);
    // outfile1 << double(averageErrorAccept1.size()) / TotalTestCases << endl;

    // outfile2.open(PROJECT_PATH + "CompareWithBaseline/ResultFiles/time_task_number.txt", std::ios_base::app);
    // outfile2 << Average(runTimeAll) << endl;
    // // if (debugMode == 1)
    // // {
    // std::cout << endl;
    // std::cout << "Files that failed during optimization:\n";
    // for (auto &file : errorFiles)
    //     std::cout << file << endl;
    // // }
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