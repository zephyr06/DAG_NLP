#include <iostream>
#pragma once

#include <fstream>
#include <dirent.h>
#include <sys/types.h>
#include <chrono>

#include "Optimize.h"
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
    vector<double> energySaveRatioVec;
    vector<double> runTime;

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
            auto res = DAG_SPACE::OptimizeScheduling(dagTasks);
            // cout << "The energy saving ratio is " << res << endl;
            auto stop = chrono::high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            double timeTaken = double(duration.count()) / 1e6;
            if (res.first >= 0 && res.first <= optimizeNLP_ErrorTolerance)
            {
                // energySaveRatioVec.push_back(res);
                runTime.push_back(timeTaken);
                outfileWrite << energySaveRatioVec.back() << endl;
            }
            else if (res.first == -1 || res.first > 1)
            {
                errorFiles.push_back(file);
            }
        }
    }

    double avEnergy = -1;
    double aveTime = -1;
    int n = runTime.size();
    if (n != 0)
    {
        avEnergy = Average(energySaveRatioVec);
        aveTime = Average(runTime);
    }

    ofstream outfile1, outfile2;
    outfile1.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/data_buffer_energy_task_number.txt", std::ios_base::app);
    outfile1 << avEnergy << endl;
    // if (debugMode)
    // {
    cout << "Average energy saving ratio is " << avEnergy << endl;
    cout << "Average time consumed is " << aveTime << endl;
    cout << "The number of tasksets under analyzation is " << energySaveRatioVec.size() << endl;
    // }

    outfile2.open("/home/zephyr/Programming/DAG_NLP/CompareWithBaseline/time_task_number.txt", std::ios_base::app);
    outfile2 << aveTime << endl;
    if (debugMode == 1)
        cout << endl;
    for (auto &file : errorFiles)
        cout << file << endl;
    // if (debugMode)
    cout << "The total number of optimization failure files is " << errorFiles.size() << endl;

    return;
}