#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Core>
// #include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "sources/Tools/testMy.h"
using namespace std;

const std::string PROJECT_PATH = "/home/zephyr/Programming/DAG_NLP/";
// const std::string PROJECT_PATH = "/home/dong/workspace/DAG_NLP/";
// const std::string PROJECT_PATH = "/home/zephyr/Programming/batch_test_DAG_NLP/VerucchiOnly/N3_10/";
//  std::string path = PROJECT_PATH + "sources/parameters.yaml";
cv::FileStorage ConfigParameters(PROJECT_PATH + "sources/parameters.yaml", cv::FileStorage::READ);

enum InitializeMethod
{
    IndexMode,
    FixedRelativeStart,
    RM,
    RM_DAG,
    Custom_DAG,
    ListScheduling
};
InitializeMethod Transform_enum(int a)
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

const double barrierBase = (double)ConfigParameters["barrierBase"];
const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const int timeScaleFactor = (int)ConfigParameters["timeScaleFactor"];

const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
double makespanWeight = (double)ConfigParameters["makespanWeight"];
double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double AcceptSchedulError = (double)ConfigParameters["AcceptSchedulError"];
const double parallelFactor = (double)ConfigParameters["parallelFactor"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"] * timeScaleFactor;
const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double zeroJacobianDetectTol = (double)ConfigParameters["zeroJacobianDetectTol"];
double FreshTol = (double)ConfigParameters["FreshTol"];
const double stepJacobianIteration = (double)ConfigParameters["stepJacobianIteration"];

const int PrintOutput = (int)ConfigParameters["PrintOutput"];
const int ResetInnerWeightLoopMax = (int)ConfigParameters["ResetInnerWeightLoopMax"];
int coreNumberAva = (int)ConfigParameters["coreNumberAva"];
int numericalJaobian = (int)ConfigParameters["numericalJaobian"];
const int setUseFixedLambdaFactor = (int)ConfigParameters["setUseFixedLambdaFactor"];
const int setDiagonalDamping = (int)ConfigParameters["setDiagonalDamping"];
const int RandomDrawWeightMaxLoop = (int)ConfigParameters["RandomDrawWeightMaxLoop"];

InitializeMethod initializeMethod = Transform_enum((int)ConfigParameters["initializeMethod"]);
double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"];
double weightDAG_factor = (double)ConfigParameters["weightDAG_factor"];
double weightInMpRTDA = (double)ConfigParameters["weightInMpRTDA"];
double weightInMpSf = (double)ConfigParameters["weightInMpSf"];
double weightInMpSfPunish = (double)ConfigParameters["weightInMpSfPunish"];
double weightInMpRTDAPunish = (double)ConfigParameters["weightInMpRTDAPunish"];
int64_t makeProgressTimeLimit = (int)ConfigParameters["makeProgressTimeLimit"];

double RtdaWeight = (double)ConfigParameters["RtdaWeight"];
double RandomAccept = (double)ConfigParameters["RandomAccept"];

const double ResetRandomWeightThreshold = (double)ConfigParameters["ResetRandomWeightThreshold"];
double weightSF_factor = (double)ConfigParameters["weightSF_factor"];
int TotalMethodUnderComparison = (int)ConfigParameters["TotalMethodUnderComparison"];
int processorAssignmentMode = (int)ConfigParameters["processorAssignmentMode"];
int PrintInitial = (int)ConfigParameters["PrintInitial"];
int debugMode = (int)ConfigParameters["debugMode"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const int TaskSetType = (int)ConfigParameters["TaskSetType"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
int tightEliminate = (int)ConfigParameters["tightEliminate"];
int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];
int whetherRandomNoiseModelSigma = (int)ConfigParameters["whetherRandomNoiseModelSigma"];
int whether_ls = (int)ConfigParameters["whether_ls"];

int whether_shuffle_CE_chain = (int)ConfigParameters["whether_shuffle_CE_chain"];
int NumCauseEffectChain = (int)ConfigParameters["NumCauseEffectChain"];

const int maxIterations = (int)ConfigParameters["maxIterations"];
const int saveGraph = (int)ConfigParameters["saveGraph"];
const int recordActionValue = (int)ConfigParameters["recordActionValue"];
int recordRLFileCount = 0;
const int MaxEliminateDetectIterations = (int)ConfigParameters["MaxEliminateDetectIterations"];
string priorityMode = (string)ConfigParameters["priorityMode"];
const string runMode = (string)ConfigParameters["runMode"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
double DataAgeThreshold = (double)ConfigParameters["DataAgeThreshold"];
double ReactionTimeThreshold = (double)ConfigParameters["ReactionTimeThreshold"];
double RoundingThreshold = (double)ConfigParameters["RoundingThreshold"];

std::string verbosityLM = (std::string)ConfigParameters["verbosityLM"];
// code below is only used to show how to read vectors, but is not actually used in this project
// vector<int> readVector(string filename)
// {
//     cv::FileStorage fs;
//     fs.open(filename, cv::FileStorage::READ);
//     cv::Mat Tt;
//     fs["chain"] >> Tt;
//     int rows = Tt.rows;

//     vector<int> vec;
//     vec.reserve(rows);
//     for (int i = 0; i < rows; i++)
//     {
//         vec.push_back(Tt.at<int>(i, 0));
//     }
//     return vec;
// }
// vector<int> CA_CHAIN = readVector(PROJECT_PATH + "sources/parameters.yaml");

const int64_t kVerucchiTimeLimit = (int)ConfigParameters["kVerucchiTimeLimit"];
const double kVerucchiReactionCost = (double)ConfigParameters["kVerucchiReactionCost"];
const double kVerucchiMaxReaction = (double)ConfigParameters["kVerucchiMaxReaction"];
const double kVerucchiDataAgeCost = (double)ConfigParameters["kVerucchiDataAgeCost"];
const double kVerucchiMaxDataAge = (double)ConfigParameters["kVerucchiMaxDataAge"];
const double kVerucchiCoreCost = (double)ConfigParameters["kVerucchiCoreCost"];

const int64_t kWangRtss21IcNlpTimeLimit = (int)ConfigParameters["kWangRtss21IcNlpTimeLimit"];

double kCplexInequalityThreshold = (double)ConfigParameters["kCplexInequalityThreshold"];

int doScheduleOptimization = (int)ConfigParameters["doScheduleOptimization"];
int considerSensorFusion = (int)ConfigParameters["considerSensorFusion"];
int doScheduleOptimizationOnlyOnce = (int)ConfigParameters["doScheduleOptimizationOnlyOnce"];