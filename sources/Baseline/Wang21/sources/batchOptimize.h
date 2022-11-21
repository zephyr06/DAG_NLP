#pragma once
#include <iostream>

#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "Optimize.h"
#include "OptimizeSA.h"
using namespace std::chrono;

namespace RTSS21IC_NLP
{

    double Average(std::vector<double> &data)
    {
        double sum = 0;
        for (int i = 0; i < int(data.size()); i++)
            sum += data[i];
        return sum / data.size();
    }

    std::vector<std::string> ReadFilesInDirectory(const char *path)
    {
        std::vector<std::string> files;
        DIR *dr;
        struct dirent *en;
        dr = opendir(path);
        if (dr)
        {
            while ((en = readdir(dr)) != NULL)
            {
                files.push_back(en->d_name); // print all directory name
            }
            closedir(dr); // close all directory
        }
        sort(files.begin(), files.end());
        return files;
    }

    void BatchOptimize()
    {
        const char *pathDataset = "/home/zephyr/Programming/DAG_NLP/TaskData/dagTasks";
        std::vector<double> averageErrorAccept;
        std::vector<double> averageErrorAccept1; // 1.0 accept
        std::vector<double> averageErrorAccept2; // 0.1 accept
        std::vector<double> errorInitial;
        std::vector<double> errorAfterOpt;
        std::vector<double> runTimeAccept;
        std::vector<double> runTimeAll;
        int N;

        std::vector<std::string> errorFiles;
        std::ofstream outfileWrite;
        outfileWrite.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt",
                          std::ios_base::app);
        std::vector<std::string> files = ReadFilesInDirectory(pathDataset);
        int TotalTestCases = files.size() - 2;
        for (const auto &file : files)
        {
            // if (GlobalVariablesDAGOpt::debugMode)
            if (batchTestMethod == 1)
                std::cout << file << std::endl;
            string delimiter = "-";
            if (file.substr(0, file.find(delimiter)) == "dag")
            {
                string path = "/home/zephyr/Programming/DAG_NLP/TaskData/dagTasks/" + file;
                OrderOptDAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(path, readTaskMode);
                N = dagTasks.tasks.size();
                auto start = std::chrono::high_resolution_clock::now();
                DAG_SPACE::OptimizeResult res;
                if (batchTestMethod == 1)
                {
                    res = DAG_SPACE::OptimizeScheduling(dagTasks);
                }
                else if (batchTestMethod == 2)
                {
                    res = DAG_SPACE::OptimizeSchedulingSA(dagTasks);
                }
                else if (batchTestMethod == 0)
                {
                    res = DAG_SPACE::InitialScheduling(dagTasks);
                }

                auto stop = std::chrono::high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                double timeTaken = double(duration.count()) / 1e6;
                runTimeAll.push_back(timeTaken);
                errorInitial.push_back(sqrt(res.initialError * 2));
                errorAfterOpt.push_back(sqrt(res.optimizeError * 2));

                if (res.optimizeError >= 0 && res.optimizeError <= AcceptSchedulError * timeScaleFactor)
                {
                    averageErrorAccept1.push_back(res.optimizeError);
                }
                if (res.optimizeError >= 0 && res.optimizeError <= 0.1 * AcceptSchedulError * timeScaleFactor)
                {
                    averageErrorAccept2.push_back(res.optimizeError);
                }

                if (res.optimizeError >= 0 && res.optimizeError <= AcceptSchedulError * timeScaleFactor)
                {
                    averageErrorAccept.push_back(res.optimizeError);
                    runTimeAccept.push_back(timeTaken);
                    outfileWrite << averageErrorAccept.back() << std::endl;
                }
                else if (res.optimizeError == -1 || res.optimizeError > AcceptSchedulError * timeScaleFactor)
                {
                    errorFiles.push_back(file);
                }
            }
        }

        double avEnergy = -1;
        double aveTime = -1;
        double averrorAfterOpt = -1;
        double avInitialError = Average(errorInitial);
        double avTimeall = Average(runTimeAll);
        int n = runTimeAccept.size();
        if (n != 0)
        {
            avEnergy = Average(averageErrorAccept);
            aveTime = Average(runTimeAccept);
            averrorAfterOpt = Average(errorAfterOpt);
        }

        std::ofstream outfile1, outfile2;
        outfile1.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/ResultFiles/utilization.txt", std::ios_base::app);
        outfile1 << double(averageErrorAccept1.size()) / TotalTestCases << std::endl;

        outfile2.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/ResultFiles/time_task_number.txt", std::ios_base::app);
        outfile2 << Average(runTimeAll) << std::endl;
        if (debugMode == 1)
        {
            std::cout << std::endl;
            std::cout << "Files that failed during optimization:\n";
            for (auto &file : errorFiles)
                std::cout << file << std::endl;
        }
        std::cout << Color::green << "Average error after optimization (accepted) is " << avEnergy << Color::def << std::endl;
        std::cout << Color::green << "Average time consumed (accepted) is " << aveTime << Color::def << std::endl;
        std::cout << Color::green << "The number of tasksets under analyzation is " << averageErrorAccept.size() << Color::def << std::endl;
        std::cout << Color::green << "Total test cases: " << TotalTestCases << Color::def << std::endl;
        std::cout << Color::blue << "Accept rate (Tol=1.0)" << double(averageErrorAccept1.size()) / TotalTestCases << Color::def << std::endl;
        std::cout << Color::blue << "Accept rate (Tol=0.1)" << double(averageErrorAccept2.size()) / TotalTestCases << Color::def << std::endl;
        std::cout << std::endl;

        std::cout << Color::blue << "Accept rate " << double(averageErrorAccept.size()) / TotalTestCases << Color::def << std::endl;
        std::cout << Color::blue << "Average initial error (all tasks) is " << avInitialError << Color::def << std::endl;
        std::cout << Color::blue << "Average error after optimization (all tasks) is " << averrorAfterOpt << Color::def << std::endl;
        std::cout << Color::blue << "Average time consumed (all) is " << avTimeall << Color::def << std::endl;

        std::cout << Color::def;
        return;
    }
} // namespace RTSS21IC_NLP
