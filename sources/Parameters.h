#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "testMy.h"
cv::FileStorage ConfigParameters("/home/zephyr/Programming/DAG_NLP/sources/parameters.yaml", cv::FileStorage::READ);
using namespace std;
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
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const int timeScaleFactor = (int)ConfigParameters["timeScaleFactor"];

const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
double makespanWeight = (double)ConfigParameters["makespanWeight"];
const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double AcceptSchedulError = (double)ConfigParameters["AcceptSchedulError"];
const double parallelFactor = (double)ConfigParameters["parallelFactor"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"] * timeScaleFactor;
const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double zeroJacobianDetectTol = (double)ConfigParameters["zeroJacobianDetectTol"];
const double FreshTol = (double)ConfigParameters["FreshTol"];
const double stepJacobianIteration = (double)ConfigParameters["stepJacobianIteration"];

const int moreElimination = (int)ConfigParameters["moreElimination"];
const int ElimnateLoop_Max = (int)ConfigParameters["ElimnateLoop_Max"];

int numericalJaobian = (int)ConfigParameters["numericalJaobian"];
const int setUseFixedLambdaFactor = (int)ConfigParameters["setUseFixedLambdaFactor"];
InitializeMethod initializeMethod = Transform_enum((int)ConfigParameters["initializeMethod"]);
const double weightPrior_factor = (double)ConfigParameters["weightPrior_factor"];
const double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"];
int debugMode = (int)ConfigParameters["debugMode"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int batchTestMethod = (int)ConfigParameters["batchTestMethod"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const int overlapMode = (int)ConfigParameters["overlapMode"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const int TaskSetType = (int)ConfigParameters["TaskSetType"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
int tightEliminate = (int)ConfigParameters["tightEliminate"];
int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];
const int maxIterations = (int)ConfigParameters["maxIterations"];
const int maxJacobianIteration = (int)ConfigParameters["maxJacobianIteration"];
string priorityMode = (string)ConfigParameters["priorityMode"];
const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const string runMode = (string)ConfigParameters["runMode"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
const int PrintOutput = (int)ConfigParameters["PrintOutput"];