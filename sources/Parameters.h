#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
cv::FileStorage ConfigParameters("/home/zephyr/Programming/DAG_NLP/sources/parameters.yaml", cv::FileStorage::READ);
using namespace std;


const double barrierBase = (double)ConfigParameters["barrierBase"];
const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
const double deltaEliminator = (double)ConfigParameters["deltaEliminator"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double initialRHS = (double)ConfigParameters["initialRHS"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double makespanWeight = (double)ConfigParameters["makespanWeight"];
const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double optimizeNLP_ErrorTolerance = (double)ConfigParameters["optimizeNLP_ErrorTolerance"];
const double parallelFactor = (double)ConfigParameters["parallelFactor"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
const double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const int debugMode = (int)ConfigParameters["debugMode"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int granularityInBF = (int)ConfigParameters["granularityInBF"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const int overlapMode = (int)ConfigParameters["overlapMode"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const string runMode = (string)ConfigParameters["runMode"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
double minWeightToBegin = (double)ConfigParameters["minWeightToBegin"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
double weight_U = (double)ConfigParameters["weight_U"];
