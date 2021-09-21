#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
cv::FileStorage ConfigParameters("/home/zephyr/Programming/DAG_NLP/sources/parameters.yaml", cv::FileStorage::READ);
using namespace std;

const double barrierBase = (double)ConfigParameters["barrierBase"];
const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
double makespanWeight = (double)ConfigParameters["makespanWeight"];
const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double optimizeNLP_ErrorTolerance = (double)ConfigParameters["optimizeNLP_ErrorTolerance"];
const double parallelFactor = (double)ConfigParameters["parallelFactor"];
const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"];
const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
const double upperLambda = (double)ConfigParameters["upperLambda"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double zeroJacobianDetectTol = (double)ConfigParameters["zeroJacobianDetectTol"];

const double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"];
int debugMode = (int)ConfigParameters["debugMode"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const int overlapMode = (int)ConfigParameters["overlapMode"];
const int randomInitialize = (int)ConfigParameters["randomInitialize"];
const int SA_iteration = (int)ConfigParameters["SA_iteration"];
const int temperatureSA = (int)ConfigParameters["temperatureSA"];
const int tightEliminate = (int)ConfigParameters["tightEliminate"];
const int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];
const int maxIterations = (int)ConfigParameters["maxIterations"];
const int maxJacobianIteration = (int)ConfigParameters["maxJacobianIteration"];

const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const string runMode = (string)ConfigParameters["runMode"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];
