#pragma once

#include <string.h>
#include "sources/Utils/Parameters.h"

namespace DAG_SPACE
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
    void WriteToResultFile(const std::string &pathDataset, const std::string &file, double res, double timeTaken, int batchTestMethod_)
    {
        std::string resFile = GetResFileName(pathDataset, file, batchTestMethod_);
        std::ofstream outfileWrite;
        outfileWrite.open(resFile,
                          std::ios_base::app);
        outfileWrite << res << std::endl;
        outfileWrite << timeTaken << std::endl;
        outfileWrite.close();
    }
    std::pair<double, double> ReadFromResultFile(const std::string &pathDataset, const std::string &file, int batchTestMethod_)
    {
        std::string resFile = GetResFileName(pathDataset, file, batchTestMethod_);
        std::ifstream cResultFile(resFile.data());
        double timeTaken = 0, res = 0;
        cResultFile >> res >> timeTaken;
        cResultFile.close();
        return std::make_pair(res, timeTaken);
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