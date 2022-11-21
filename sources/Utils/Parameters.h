#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Core>
// #include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "sources/Utils/testMy.h"
// using namespace std;
// All the global variables should be const

const std::string PROJECT_PATH = "/home/zephyr/Programming/DAG_NLP/";
// const std::string PROJECT_PATH = "/home/dong/workspace/DAG_NLP/";
// const std::string PROJECT_PATH = "/home/zephyr/Programming/batch_test_DAG_NLP/VerucchiOnly/N3_10/";
//  std::string path = PROJECT_PATH + "sources/parameters.yaml";
static cv::FileStorage ConfigParameters(PROJECT_PATH + "sources/parameters.yaml", cv::FileStorage::READ);
// TODO: add a namespace for these parameters, probably GlobalVariables?
enum InitializeMethod
{
    IndexMode,
    FixedRelativeStart,
    RM,
    RM_DAG,
    Custom_DAG,
    ListScheduling
};
inline InitializeMethod Transform_enum(int a)
{
    if (a == 0)
        return IndexMode;
    else if (a == 1)
        return FixedRelativeStart;
    else if (a == 2)
        return RM;
    else if (a == 3)
        return RM_DAG;
    else if (a == 4)
        return Custom_DAG;
    else if (a == 5)
        return ListScheduling;
    else
    {
        CoutError("Not recognized enum InitializeMethod\n");
        return IndexMode;
    }
}

static const double barrierBase = (double)ConfigParameters["barrierBase"];
static const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
static const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
static double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"]; // not used in the main SF optimization program
static const int timeScaleFactor = (int)ConfigParameters["timeScaleFactor"];

static const double initialLambda = (double)ConfigParameters["initialLambda"];
static const double lowerLambda = (double)ConfigParameters["lowerLambda"];
static double makespanWeight = (double)ConfigParameters["makespanWeight"];   // not used in the main SF optimization program
static double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"]; // not used in the main SF optimization program
static const double AcceptSchedulError = (double)ConfigParameters["AcceptSchedulError"];
static const double parallelFactor = (double)ConfigParameters["parallelFactor"];
static const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];

static const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
static const double upperLambda = (double)ConfigParameters["upperLambda"];
static const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
static const double zeroJacobianDetectTol = (double)ConfigParameters["zeroJacobianDetectTol"];
static const double stepJacobianIteration = (double)ConfigParameters["stepJacobianIteration"];

static int printSchedule = (int)ConfigParameters["printSchedule"];
static int bigJobGroupCheck = (int)ConfigParameters["bigJobGroupCheck"];
static const int PrintOutput = (int)ConfigParameters["PrintOutput"];
static const int ResetInnerWeightLoopMax = (int)ConfigParameters["ResetInnerWeightLoopMax"];
static const int coreNumberAva = (int)ConfigParameters["coreNumberAva"];
static int numericalJaobian = (int)ConfigParameters["numericalJaobian"]; // not used in the main SF optimization program
static const int setUseFixedLambdaFactor = (int)ConfigParameters["setUseFixedLambdaFactor"];
static const int setDiagonalDamping = (int)ConfigParameters["setDiagonalDamping"];
static const int RandomDrawWeightMaxLoop = (int)ConfigParameters["RandomDrawWeightMaxLoop"];

// ********************* Under experiment or not useful
static int enableFastSearch = (int)ConfigParameters["enableFastSearch"];
static int enableSmallJobGroupCheck = (int)ConfigParameters["enableSmallJobGroupCheck"];
static const int subJobGroupMaxSize = (int)ConfigParameters["subJobGroupMaxSize"];

// ********************* TO-FIX:
static double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"] * timeScaleFactor;
static double freshTol = (double)ConfigParameters["freshTol"];

static InitializeMethod initializeMethod = Transform_enum((int)ConfigParameters["initializeMethod"]);
static double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"]; // not used in the main SF optimization program
static double weightDAG_factor = (double)ConfigParameters["weightDAG_factor"]; // not used in the main SF optimization program
static const double weightInMpRTDA = (double)ConfigParameters["weightInMpRTDA"];
static const double weightInMpSf = (double)ConfigParameters["weightInMpSf"];
static const double weightInMpSfPunish = (double)ConfigParameters["weightInMpSfPunish"];
static const double weightInMpRTDAPunish = (double)ConfigParameters["weightInMpRTDAPunish"];
static const int64_t makeProgressTimeLimit = (int)ConfigParameters["makeProgressTimeLimit"];

static const double RtdaWeight = (double)ConfigParameters["RtdaWeight"];
// const double RandomAccept = (double)ConfigParameters["RandomAccept"];

static const double ResetRandomWeightThreshold = (double)ConfigParameters["ResetRandomWeightThreshold"];
static double weightSF_factor = (double)ConfigParameters["weightSF_factor"]; // not used in the main SF optimization program
static const int TotalMethodUnderComparison = (int)ConfigParameters["TotalMethodUnderComparison"];
static const int processorAssignmentMode = (int)ConfigParameters["processorAssignmentMode"];
static const int PrintInitial = (int)ConfigParameters["PrintInitial"];
static int debugMode = (int)ConfigParameters["debugMode"]; // why can't it be const?
// int debugMode=0;
static const int exactJacobian = (int)ConfigParameters["exactJacobian"];
static const int optimizerType = (int)ConfigParameters["optimizerType"];
static const int randomInitialize = (int)ConfigParameters["randomInitialize"];
static const int SA_iteration = (int)ConfigParameters["SA_iteration"];
static const int TaskSetType = (int)ConfigParameters["TaskSetType"];
static const int temperatureSA = (int)ConfigParameters["temperatureSA"];
static int tightEliminate = (int)ConfigParameters["tightEliminate"];                             // not used in the main SF optimization program
static int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];     // not used in the main SF optimization program
static int whetherRandomNoiseModelSigma = (int)ConfigParameters["whetherRandomNoiseModelSigma"]; // not used in the main SF optimization program
static int whether_ls = (int)ConfigParameters["whether_ls"];                                     // not used in the main SF optimization program

static const int whether_shuffle_CE_chain = (int)ConfigParameters["whether_shuffle_CE_chain"];
static const int NumCauseEffectChain = (int)ConfigParameters["NumCauseEffectChain"];

static const int maxIterations = (int)ConfigParameters["maxIterations"];
static const int saveGraph = (int)ConfigParameters["saveGraph"];
static const int recordActionValue = (int)ConfigParameters["recordActionValue"];
// int recordRLFileCount = 0;
static const int MaxEliminateDetectIterations = (int)ConfigParameters["MaxEliminateDetectIterations"];
static const std::string priorityMode = (std::string)ConfigParameters["priorityMode"];
static const std::string runMode = (std::string)ConfigParameters["runMode"];
static const std::string testDataSetName = (std::string)ConfigParameters["testDataSetName"];
static const double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
static const double DataAgeThreshold = (double)ConfigParameters["DataAgeThreshold"];
static const double ReactionTimeThreshold = (double)ConfigParameters["ReactionTimeThreshold"];
static const double RoundingThreshold = (double)ConfigParameters["RoundingThreshold"];

static const std::string verbosityLM = (std::string)ConfigParameters["verbosityLM"];
// code below is only used to show how to read vectors, but is not actually used in this project
// std::vector<int> readVector(std::string filename)
// {
//     cv::FileStorage fs;
//     fs.open(filename, cv::FileStorage::READ);
//     cv::Mat Tt;
//     fs["chain"] >> Tt;
//     int rows = Tt.rows;

//    std::vector<int> vec;
//     vec.reserve(rows);
//     for (int i = 0; i < rows; i++)
//     {
//         vec.push_back(Tt.at<int>(i, 0));
//     }
//     return vec;
// }
// std::vector<int> CA_CHAIN = readVector(PROJECT_PATH + "sources/parameters.yaml");

static const int64_t kVerucchiTimeLimit = (int)ConfigParameters["kVerucchiTimeLimit"];
static const double kVerucchiReactionCost = (double)ConfigParameters["kVerucchiReactionCost"];
static const double kVerucchiMaxReaction = (double)ConfigParameters["kVerucchiMaxReaction"];
static const double kVerucchiDataAgeCost = (double)ConfigParameters["kVerucchiDataAgeCost"];
static const double kVerucchiMaxDataAge = (double)ConfigParameters["kVerucchiMaxDataAge"];
static const double kVerucchiCoreCost = (double)ConfigParameters["kVerucchiCoreCost"];

static const int64_t kWangRtss21IcNlpTimeLimit = (int)ConfigParameters["kWangRtss21IcNlpTimeLimit"];

static const double kCplexInequalityThreshold = (double)ConfigParameters["kCplexInequalityThreshold"];

static const int doScheduleOptimization = (int)ConfigParameters["doScheduleOptimization"];
static int considerSensorFusion = (int)ConfigParameters["considerSensorFusion"]; // previous code heavily depended on its value, but we must treat it as a const int in the future
static const int doScheduleOptimizationOnlyOnce = (int)ConfigParameters["doScheduleOptimizationOnlyOnce"];
static const int BatchTestOnlyOneMethod = (int)ConfigParameters["BatchTestOnlyOneMethod"];
