#pragma once
#include <iostream>

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

    OrderOptDAG_SPACE::ScheduleResult PerformSingleScheduling(DAG_Model &dagTasks, OptimizeSF::ScheduleOptions &scheduleOptions, BaselineMethods batchTestMethod)
    {
        ScheduleResult res;
        switch (batchTestMethod)
        {
        case InitialMethod:
            res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT(dagTasks, scheduleOptions.processorNum_, GlobalVariablesDAGOpt::sensorFusionTolerance, GlobalVariablesDAGOpt::freshTol);
            break;
        case Verucchi20:
            if (GlobalVariablesDAGOpt::considerSensorFusion != 0)
                CoutError("ScheduleVerucchiRTDA is called with non-zero considerSensorFusion!");
            res = ScheduleVerucchiRTDA(dagTasks, dagTasks.chains_, scheduleOptions.processorNum_, 15.0, 400000.0, 15.0, 400000.0, 15.0);
            break;
        case Wang21:
            if (GlobalVariablesDAGOpt::considerSensorFusion == 0)
                CoutError("ScheduleRTSS21IC is called with zero considerSensorFusion!");
            res = OrderOptDAG_SPACE::ScheduleRTSS21IC(dagTasks, GlobalVariablesDAGOpt::sensorFusionTolerance, GlobalVariablesDAGOpt::freshTol);
            break;
        case TOM:
            scheduleOptions.doScheduleOptimization_ = 1;
            scheduleOptions.doScheduleOptimizationOnlyOnce_ = 0;
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(dagTasks, scheduleOptions);
            break;

        case TOM_Fast:
            scheduleOptions.doScheduleOptimization_ = 0;
            scheduleOptions.doScheduleOptimizationOnlyOnce_ = 0;
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(dagTasks, scheduleOptions);
            break;

        case TOM_FastLP:
            scheduleOptions.doScheduleOptimization_ = 0;
            scheduleOptions.doScheduleOptimizationOnlyOnce_ = 1;
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(dagTasks, scheduleOptions);
            break;
        default:
            CoutError("Please provide batchTestMethod implementation!");
        }
        return res;
    }

    struct BatchResult
    {
        double schedulableRatio;
        double performance;
        double runTime;
    };

    std::vector<BatchResult> BatchOptimizeOrder(std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods, std::string dataSetFolder = PROJECT_PATH + "TaskData/dagTasks/")
    {
        std::string dirStr = dataSetFolder;
        const char *pathDataset = (dirStr).c_str();
        std::cout << "Dataset Directory: " << pathDataset << std::endl;
        std::vector<std::vector<double>> runTimeAll(GlobalVariablesDAGOpt::TotalMethodUnderComparison);
        std::vector<std::vector<double>> objsAll(GlobalVariablesDAGOpt::TotalMethodUnderComparison);
        std::vector<std::vector<int>> schedulableAll(GlobalVariablesDAGOpt::TotalMethodUnderComparison); // values could only be 0 / 1

        std::vector<std::string> errorFiles;
        std::vector<std::string> worseFiles;
        std::vector<std::string> files = ReadFilesInDirectory(pathDataset);
        for (const auto &file : files)
        {
            std::string delimiter = "-";
            if (file.substr(0, file.find(delimiter)) == "dag" && file.find("Res") == std::string::npos)
            {
                std::cout << file << std::endl;
                std::string path = PROJECT_PATH + "TaskData/dagTasks/" + file;
                OrderOptDAG_SPACE::DAG_Model dagTasks = OrderOptDAG_SPACE::ReadDAG_Tasks(path, GlobalVariablesDAGOpt::priorityMode);
                if (dagTasks.GetSfBound() > 0)
                    GlobalVariablesDAGOpt::sensorFusionTolerance = dagTasks.GetSfBound();
                if (dagTasks.GetRtdaBound() > 0)
                    GlobalVariablesDAGOpt::freshTol = dagTasks.GetRtdaBound();

                // int N = dagTasks.tasks.size();
                AssertBool(true, dagTasks.chains_.size() > 0, __LINE__);
                for (auto batchTestMethod : baselineMethods)
                {
                    OrderOptDAG_SPACE::ScheduleResult res;
                    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
                    scheduleOptions.LoadParametersYaml();
                    if (VerifyResFileExist(pathDataset, file, batchTestMethod))
                    {
                        res = ReadFromResultFile(pathDataset, file, batchTestMethod);
                    }
                    else
                    {
                        auto start = std::chrono::high_resolution_clock::now();
                        res = PerformSingleScheduling(dagTasks, scheduleOptions, batchTestMethod);
                        auto stop = std::chrono::high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(stop - start);
                        res.timeTaken_ = double(duration.count()) / 1e6;
                    }
                    std::cout << "Schedulable? " << res.schedulable_ << std::endl;
                    std::cout << "Objective: " << res.obj_ << std::endl;

                    if (res.schedulable_ == false)
                    {
                        if (objsAll[0].size() == 0)
                            CoutError("Initial method is not feasible in " + file);
                        res.obj_ = objsAll[0].back();
                        if (batchTestMethod >= 3)
                            errorFiles.push_back(file);
                    }
                    WriteToResultFile(pathDataset, file, res, batchTestMethod);
                    if (res.schedulable_ == true && res.startTimeVector_.cols() > 0 && res.startTimeVector_.rows() > 0)
                        WriteScheduleToFile(pathDataset, file, dagTasks, res, batchTestMethod);

                    runTimeAll[batchTestMethod].push_back(res.timeTaken_);
                    schedulableAll[batchTestMethod].push_back((res.schedulable_ ? 1 : 0));
                    objsAll[batchTestMethod].push_back(res.obj_);
                }
                if (objsAll[2].size() > 0 && objsAll[1].back() > objsAll[2].back())
                {
                    CoutWarning("One case where proposed method performs worse is found: " + file);
                    worseFiles.push_back(file);
                }
            }
        }

        int n = objsAll[0].size();
        if (n != 0)
        {
            VariadicTable<std::string, double, double, double> vt({"Method", "Schedulable ratio", "Obj (Only used in RTDA experiment)", "TimeTaken"}, 10);

            vt.addRow("Initial", Average(schedulableAll[0]), Average(objsAll[0]), Average(runTimeAll[0]));
            vt.addRow("Verucchi20", Average(schedulableAll[1]), Average(objsAll[1]), Average(runTimeAll[1]));
            vt.addRow("Wang21", Average(schedulableAll[2]), Average(objsAll[2]), Average(runTimeAll[2]));
            vt.addRow("TOM", Average(schedulableAll[3]), Average(objsAll[3]), Average(runTimeAll[3]));
            vt.addRow("TOM_Fast", Average(schedulableAll[4]), Average(objsAll[4]), Average(runTimeAll[4]));
            vt.addRow("TOM_FastLP", Average(schedulableAll[5]), Average(objsAll[5]), Average(runTimeAll[5]));
            vt.print(std::cout);
        }

        std::cout << "The number of error files: " << errorFiles.size() << std::endl;
        for (std::string file : errorFiles)
            std::cout << file << std::endl;

        std::cout << "The number of files where OrderOpt performs worse: " << worseFiles.size() << std::endl;
        for (std::string file : worseFiles)
            std::cout << file << std::endl;

        std::vector<BatchResult> batchResVec;
        for (auto method : baselineMethods)
        {
            int i = method;
            BatchResult batchRes{Average(schedulableAll[i]), Average(objsAll[i]), Average(runTimeAll[i])};
            batchResVec.push_back(batchRes);
        }

        return batchResVec;
    }
} // namespace OrderOptDAG_SPACE