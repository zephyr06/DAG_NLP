#pragma once

#include <string.h>
#include "sources/Utils/Parameters.h"
#include "sources/Optimization/OptimizeOrder.h"

namespace OrderOptDAG_SPACE
{

    inline std::string GetResFileName(const std::string &pathDataset, const std::string &file, int batchTestMethod_)
    {
        std::string property;
        if (batchTestMethod_ == 0)
        {
            property = "_Initial_Res.txt";
        }
        else if (batchTestMethod_ == 1)
        {
            property = "_OptOrder_Res.txt";
        }
        else if (batchTestMethod_ == 2)
        {
            property = "_Verucchi_Res.txt";
        }
        else if (batchTestMethod_ == 3)
        {
            property = "SA_Res.txt";
        }
        return pathDataset + file + property;
    }
    // TOTEST: read & write
    void WriteToResultFile(const std::string &pathDataset, const std::string &file, OrderOptDAG_SPACE::ScheduleResult &res, int batchTestMethod_)
    {
        std::string resFile = GetResFileName(pathDataset, file, batchTestMethod_);
        std::ofstream outfileWrite;
        outfileWrite.open(resFile, std::ofstream::out | std::ofstream::trunc); // std::ios_base::app
        outfileWrite << res.schedulable_ << std::endl;
        outfileWrite << res.obj_ << std::endl;
        outfileWrite << res.timeTaken_ << std::endl;
        outfileWrite.close();
    }

    OrderOptDAG_SPACE::ScheduleResult ReadFromResultFile(const std::string &pathDataset, const std::string &file, int batchTestMethod_)
    {
        OrderOptDAG_SPACE::ScheduleResult result;
        std::string resFile = GetResFileName(pathDataset, file, batchTestMethod_);
        std::ifstream cResultFile(resFile.data());
        // double timeTaken = 0, obj = 0;
        // int schedulable = 0;
        cResultFile >> result.schedulable_ >> result.obj_ >> result.timeTaken_;
        cResultFile.close();
        return result;
    }

    bool VerifyResFileExist(const std::string &pathDataset, const std::string &file, int batchTestMethod_)
    {
        std::string resFile = GetResFileName(pathDataset, file, batchTestMethod_);
        std::ifstream myfile;
        myfile.open(resFile);
        if (myfile)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    template <typename T>
    double Average(std::vector<T> &data)
    {
        if (data.size())
        {
            T sum = 0;
            for (int i = 0; i < int(data.size()); i++)
                sum += data[i];
            return double(sum) / data.size();
        }
        else
        {
            return -1;
        }
    }
}