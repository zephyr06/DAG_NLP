#pragma once
#include <dirent.h>
#include <sys/types.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include "assert.h"
#include "sources/Utils/VariadicTable.h"

// #include "sources/Optimization/Optimize.h"
#include "sources/Baseline/RTSS21IC.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Optimization/GlobalOptimization.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/OptimizeSFOrder_TOM.h"
#include "sources/Optimization/OptimizeSFOrder_TOM_IA.h"
#include "sources/Utils/BatchUtils.h"
#include "sources/Utils/ScheduleResults.h"

using namespace std::chrono;
namespace OrderOptDAG_SPACE {

void ClearResultFiles(std::string dataSetFolder);

template <typename ObjectiveFunctionBase>
OrderOptDAG_SPACE::ScheduleResult PerformSingleScheduling(
    DAG_Model &dagTasks, OptimizeSF::ScheduleOptions &scheduleOptions,
    BASELINEMETHODS batchTestMethod) {
    ScheduleResult res;
    switch (batchTestMethod) {
        case InitialMethod:
            res = OrderOptDAG_SPACE::ScheduleDAGLS_LFT<ObjectiveFunctionBase>(
                dagTasks, scheduleOptions);
            break;
        case Verucchi20:
            res = ScheduleVerucchiRTDA(dagTasks, dagTasks.chains_,
                                       scheduleOptions.processorNum_,
                                       ObjectiveFunctionBase::type_trait);
            break;
        case Wang21:  // should not be used
            CoutError("Wang21 is not used in current experiments!");
            // res = OrderOptDAG_SPACE::ScheduleRTSS21IC(
            //     dagTasks, GlobalVariablesDAGOpt::sensorFusionTolerance,
            //     GlobalVariablesDAGOpt::freshTol);
            break;
        case TOM:
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<
                LPOrderScheduler, ObjectiveFunctionBase>(dagTasks,
                                                         scheduleOptions);
            break;

        case TOM_IA:
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel_IA<
                LPOrderScheduler, ObjectiveFunctionBase>(dagTasks,
                                                         scheduleOptions);
            break;

        case TOM_Fast:
            res = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<
                SimpleOrderScheduler, ObjectiveFunctionBase>(dagTasks,
                                                             scheduleOptions);
            break;

        case GlobalOpt: {  // should only be used in a few situations
            PermutationStatus permSta =
                FindGlobalOptRTDA(dagTasks, scheduleOptions);
            res = permSta.GetScheduleResult();
            if (permSta.whetherTimeOut)
                res.discardResult_ = true;
            break;
        }

        default:
            CoutError("Please provide batchTestMethod implementation!");
    }
    return res;
}

template <typename ObjectiveFunctionBase>
std::unordered_map<OrderOptDAG_SPACE::BASELINEMETHODS, BatchResult>
BatchOptimizeOrder(
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> &baselineMethods,
    std::string dataSetFolder = GlobalVariablesDAGOpt::PROJECT_PATH +
                                "TaskData/dagTasks/") {
    std::string dirStr = dataSetFolder;
    const char *pathDataset = (dirStr).c_str();
    std::cout << "Dataset Directory: " << pathDataset << std::endl;

    // Prepare intermediate data records
    ResultsManager results_man(baselineMethods);
    std::vector<bool> validFileIndex(1000, true);
    std::vector<std::string> errorFiles;

    std::vector<std::string> files = ReadFilesInDirectory(pathDataset);
    int fileIndex = 0;
    for (const auto &file : files) {
        std::string delimiter = "-";
        if (file.substr(0, file.find(delimiter)) == "dag" &&
            file.find("Res") == std::string::npos &&
            file.find("LoopCount") == std::string::npos) {
            std::cout << file << std::endl;
            std::string path = dataSetFolder + file;
            OrderOptDAG_SPACE::DAG_Model dagTasks =
                OrderOptDAG_SPACE::ReadDAG_Tasks(
                    path, GlobalVariablesDAGOpt::priorityMode);

            for (auto batchTestMethod : baselineMethods) {
                OrderOptDAG_SPACE::ScheduleResult res;
                OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOptions;
                scheduleOptions.LoadParametersYaml();

                if (VerifyResFileExist(pathDataset, file, batchTestMethod,
                                       ObjectiveFunctionBase::type_trait)) {
                    res = ReadFromResultFile(pathDataset, file, batchTestMethod,
                                             ObjectiveFunctionBase::type_trait);
                } else {
                    auto start = std::chrono::high_resolution_clock::now();
                    res = PerformSingleScheduling<ObjectiveFunctionBase>(
                        dagTasks, scheduleOptions, batchTestMethod);
                    auto stop = std::chrono::high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>(stop - start);
                    res.timeTaken_ = double(duration.count()) / 1e6;
                }
                std::cout << "Schedulable? " << res.schedulable_ << std::endl;
                std::cout << Color::green << "Objective: " << res.obj_
                          << Color::def << std::endl;
                std::cout << "res.timeTaken_: " << res.timeTaken_ << "\n\n";

                // if timeTaken_ approaches 600s and the method is GlobalOpt,
                // then we'll skip its results because cannot guarantee global
                // optimal
                if (res.timeTaken_ >
                        GlobalVariablesDAGOpt::kGlobalOptimizationTimeLimit -
                            10 &&
                    batchTestMethod == BASELINEMETHODS::GlobalOpt) {
                    validFileIndex[fileIndex] = false;
                }

                if (res.schedulable_ == false && batchTestMethod == TOM) {
                    errorFiles.push_back(file);
                }

                results_man.add(batchTestMethod, res, file);
                WriteToResultFile(pathDataset, file, res, batchTestMethod,
                                  ObjectiveFunctionBase::type_trait);
                // if (res.schedulable_ == true && res.startTimeVector_.rows() >
                // 0)
                //     WriteScheduleToFile(pathDataset, file, dagTasks, res,
                //                         batchTestMethod);
            }
            fileIndex++;
        }
    }

    // result analysis
    results_man.PrintWorseCase(BASELINEMETHODS::InitialMethod,
                               BASELINEMETHODS::TOM);
    results_man.PrintResultTable(baselineMethods);
    // if (std::find(baselineMethods.begin(), baselineMethods.end(),
    //               BASELINEMETHODS::TOM_WSkip) != baselineMethods.end())
    //     results_man.PrintLongestCase(BASELINEMETHODS::TOM_WSkip);
    results_man.PrintTimeOutCase();
    results_man.PrintTimeOutRatio();

    // int n = objsAll[0].size();
    // if (n != 0 && ObjectiveFunctionBase::type_trait == "RTDAExperimentObj") {
    //     VariadicTable<std::string, double, double, double, double> vt(
    //         {"Method", "Schedulable ratio",
    //          "Obj (Only used in RTDA experiment)", "Obj(Norm)", "TimeTaken"},
    //         10);

    //     vt.addRow("Initial", Average(schedulableAll[0]),
    //               Average(objsAll[0], validFileIndex),
    //               Average(objsAllNorm[0]), Average(runTimeAll[0]));
    //     vt.addRow("Verucchi20", Average(schedulableAll[1]),
    //     Average(objsAll[1]),
    //               Average(objsAllNorm[1]), Average(runTimeAll[1]));
    //     vt.addRow("Wang21", Average(schedulableAll[2]), Average(objsAll[2]),
    //               Average(objsAllNorm[2]), Average(runTimeAll[2]));
    //     vt.addRow("TOM", Average(schedulableAll[3]),
    //               Average(objsAll[3], validFileIndex),
    //               Average(objsAllNorm[3], validFileIndex),
    //               Average(runTimeAll[3], validFileIndex));
    //     vt.addRow("TOM_Fast", Average(schedulableAll[4]),
    //     Average(objsAll[4]),
    //               Average(objsAllNorm[4]), Average(runTimeAll[4]));
    //     vt.addRow("TOM_FastLP", Average(schedulableAll[5]),
    //     Average(objsAll[5]),
    //               Average(objsAllNorm[5]), Average(runTimeAll[5]));
    //     vt.addRow("GlobalOptimal", Average(schedulableAll[6]),
    //               Average(objsAll[6], validFileIndex),
    //               Average(objsAllNorm[6], validFileIndex),
    //               Average(runTimeAll[6], validFileIndex));
    //     vt.print(std::cout);
    // } else if (n != 0 && ObjectiveFunctionBase::type_trait == "RTSS21ICObj")
    // {
    //     VariadicTable<std::string, double, double> vt(
    //         {"Method", "Schedulable ratio", "TimeTaken"}, 10);

    //     vt.addRow("Initial", Average(schedulableAll[0]),
    //               Average(runTimeAll[0]));
    //     vt.addRow("Verucchi20", Average(schedulableAll[1]),
    //               Average(runTimeAll[1]));
    //     vt.addRow("Wang21", Average(schedulableAll[2]),
    //     Average(runTimeAll[2])); vt.addRow("TOM", Average(schedulableAll[3]),
    //     Average(runTimeAll[3])); vt.addRow("TOM_Fast",
    //     Average(schedulableAll[4]),
    //               Average(runTimeAll[4]));
    //     vt.addRow("TOM_FastLP", Average(schedulableAll[5]),
    //               Average(runTimeAll[5]));
    //     vt.print(std::cout);
    // }

    // std::cout << "The number of error files: " << errorFiles.size()
    //           << std::endl;
    // for (std::string file : errorFiles) std::cout << file << std::endl;

    // std::cout << "The number of files where OrderOpt performs worse: "
    //           << worseFiles.size() << std::endl;
    // for (std::string file : worseFiles) std::cout << file << std::endl;

    // std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethodsAll = {
    //     InitialMethod,  // 0
    //     Verucchi20,     // 1
    //     Wang21,         // 2
    //     TOM,            // 3
    //     TOM_Fast,       // 4
    //     TOM_FastLP,     // 5
    //     GlobalOpt       // 6
    // };
    // std::vector<BatchResult> batchResVec;
    // for (auto method : baselineMethodsAll) {
    //     int i = method;
    //     BatchResult batchRes{Average(schedulableAll[i]), Average(objsAll[i]),
    //                          Average(runTimeAll[i])};
    //     batchResVec.push_back(batchRes);
    // }
    // std::cout << "Case that takes the longest time in TOM: "
    //           << int((std::max_element(runTimeAll[3].begin(),
    //                                    runTimeAll[3].end())) -
    //                  runTimeAll[3].begin())
    //           << "\n";
    // return batchResVec;

    return results_man.GetBatchResVec(baselineMethods);
}
}  // namespace OrderOptDAG_SPACE