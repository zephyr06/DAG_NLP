#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

cv::FileStorage ConfigParameters("/home/zephyr/Programming/DAG_NLP/sources/parameters.yaml", cv::FileStorage::READ);

using namespace std;

const double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"];

// **************************************************************old ones
// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
// int TASK_NUMBER_DYNAMIC = 10;
int TASK_NUMBER = 0;
double weight_U = (double)ConfigParameters["weight_U"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
double minWeightToBegin = (double)ConfigParameters["minWeightToBegin"];

const int exactJacobian = (int)ConfigParameters["exactJacobian"];

const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];

const double deltaEliminator = (double)ConfigParameters["deltaEliminator"];
const double initialRHS = (double)ConfigParameters["initialRHS"];
const double makespanWeight = (double)ConfigParameters["makespanWeight"];
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double barrierBase = (double)ConfigParameters["barrierBase"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];

const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];

const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
const string runMode = (string)ConfigParameters["runMode"];

const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const int granularityInBF = (int)ConfigParameters["granularityInBF"];
const int debugMode = (int)ConfigParameters["debugMode"];
