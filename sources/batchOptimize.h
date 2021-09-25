#pragma once
#include <iostream>

#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "Optimize.h"
#include "OptimizeSA.h"
using namespace std::chrono;

double Average(vector<double> &data)
{
    double sum = 0;
    for (int i = 0; i < int(data.size()); i++)
        sum += data[i];
    return sum / data.size();
}

vector<string> ReadFilesInDirectory(const char *path)
{
    vector<string> files;
    DIR *dr;
    struct dirent *en;
    dr = opendir(path);
    if (dr)
    {
        while ((en = readdir(dr)) != NULL)
        {
            files.push_back(en->d_name); //print all directory name
        }
        closedir(dr); //close all directory
    }
    return files;
}

void BatchOptimize()
{
    const char *pathDataset = "/home/zephyr/Programming/DAG_NLP/TaskData/dagTasks";
    vector<double> averageErrorAccept;
    vector<double> errorInitial;
    vector<double> errorAfterOpt;
    vector<double> runTimeAccept;
    vector<double> runTimeAll;

    vector<string> errorFiles;
    ofstream outfileWrite;
    outfileWrite.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt",
                      std::ios_base::app);

    for (const auto &file : ReadFilesInDirectory(pathDataset))
    {
        if (debugMode)
            cout << file << endl;
        string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag")
        {
            string path = "/home/zephyr/Programming/DAG_NLP/TaskData/dagTasks/" + file;
            DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(path, readTaskMode);
            auto start = chrono::high_resolution_clock::now();
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

            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
            runTimeAll.push_back(timeTaken);
            errorInitial.push_back(sqrt(res.initialError * 2));
            errorAfterOpt.push_back(sqrt(res.optimizeError * 2));
            if (res.optimizeError >= 0 && res.optimizeError <= AcceptSchedulError)
            {
                averageErrorAccept.push_back(res.optimizeError);
                runTimeAccept.push_back(timeTaken);
                outfileWrite << averageErrorAccept.back() << endl;
            }
            else if (res.optimizeError == -1 || res.optimizeError > AcceptSchedulError)
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

    ofstream outfile1, outfile2;
    outfile1.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt", std::ios_base::app);
    outfile1 << avEnergy << endl;

    outfile2.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/time_task_number.txt", std::ios_base::app);
    outfile2 << aveTime << endl;
    if (debugMode == 1)
        cout << endl;
    for (auto &file : errorFiles)
        cout << file << endl;

    cout << Color::green << "Average error after optimization (accepted) is " << avEnergy << Color::def << endl;
    cout << Color::green << "Average time consumed (accepted) is " << aveTime << Color::def << endl;
    cout << Color::green << "The number of tasksets under analyzation is " << averageErrorAccept.size() << Color::def << endl;
    cout << Color::blue << "Accept rate " << double(averageErrorAccept.size()) / (errorFiles.size() + averageErrorAccept.size()) << Color::def << endl;
    cout << Color::blue << "Average initial error (all tasks) is " << avInitialError << Color::def << endl;
    cout << Color::blue << "Average error after optimization (all tasks) is " << averrorAfterOpt << Color::def << endl;
    cout << Color::blue << "Average time consumed (all) is " << avTimeall << Color::def << endl;

    cout << Color::def;
    return;
}