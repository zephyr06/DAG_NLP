#pragma once
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include "sources/Tools/testMy.h"
using namespace std;
string path = "/home/zephyr/Programming/DAG_NLP/sources/parameters.yaml";
cv::FileStorage ConfigParameters(path, cv::FileStorage::READ);

enum InitializeMethod
{
    IndexMode,
    FixedRelativeStart,
    RM,
    RM_DAG
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
const double weightPrior_factor = (double)ConfigParameters["weightPrior_factor"];
double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"];
double weightDAG_factor = (double)ConfigParameters["weightDAG_factor"];
double RtdaWeight = (double)ConfigParameters["RtdaWeight"];

const double ResetRandomWeightThreshold = (double)ConfigParameters["ResetRandomWeightThreshold"];

int PrintInitial = (int)ConfigParameters["PrintInitial"];
int debugMode = (int)ConfigParameters["debugMode"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int batchTestMethod = (int)ConfigParameters["batchTestMethod"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const int TaskSetType = (int)ConfigParameters["TaskSetType"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
int tightEliminate = (int)ConfigParameters["tightEliminate"];
int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];
int whetherRandomNoiseModelSigma = (int)ConfigParameters["whetherRandomNoiseModelSigma"];
int whether_ls = (int)ConfigParameters["whether_ls"];

const int maxIterations = (int)ConfigParameters["maxIterations"];
const int saveGraph = (int)ConfigParameters["saveGraph"];
const int recordActionValue = (int)ConfigParameters["recordActionValue"];
int recordRLFileCount = 0;
const int MaxEliminateDetectIterations = (int)ConfigParameters["MaxEliminateDetectIterations"];
string priorityMode = (string)ConfigParameters["priorityMode"];
const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const string runMode = (string)ConfigParameters["runMode"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
std::string verbosityLM = (std::string)ConfigParameters["verbosityLM"];
// code below is only used to show how to read vectors, but is not actually used in this project
vector<double> readVector(string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    cv::Mat Tt;
    fs["T"] >> Tt;
    int rows = Tt.rows;

    vector<double> vec;
    vec.reserve(rows);
    for (int i = 0; i < rows; i++)
    {
        vec.push_back(Tt.at<double>(i, 0));
    }
    return vec;
}
vector<double> aaa = readVector(path);
