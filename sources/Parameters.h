#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

cv::FileStorage ConfigParameters("/home/lab/Programming/Energy_Opt_NLP/sources/parameters.yaml", cv::FileStorage::READ);

using namespace std;

const double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"];

// **************************************************************old ones
// const int TASK_NUMBER = (int)ConfigParameters["TASK_NUMBER"];
// int TASK_NUMBER_DYNAMIC = 10;
int TASK_NUMBER = 0;
double weightEnergy = (double)ConfigParameters["weightEnergy"];
double punishmentInBarrier = weightEnergy * (double)ConfigParameters["punishmentInBarrier"];
double eliminateTol = (double)ConfigParameters["eliminateTol"];
double minWeightToBegin = (double)ConfigParameters["minWeightToBegin"];
int enableIPM = (int)ConfigParameters["enableIPM"];

const int enableMaxComputation = (int)ConfigParameters["enableMaxComputation"];
const int exactJacobian = (int)ConfigParameters["exactJacobian"];
const double computationBound = (double)ConfigParameters["computationBound"];

const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
const double initialLambda = (double)ConfigParameters["initialLambda"];
const double lowerLambda = (double)ConfigParameters["lowerLambda"];
const double upperLambda = (double)ConfigParameters["upperLambda"];

const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];

const int weightEnergyMaxOrder = (int)ConfigParameters["weightEnergyMaxOrder"];

const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
const double toleranceBarrier = (double)ConfigParameters["toleranceBarrier"];
const int optimizerType = (int)ConfigParameters["optimizerType"];
const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
const double punishmentFrequency = (double)ConfigParameters["punishmentFrequency"];
const string testDataSetName = (string)ConfigParameters["testDataSetName"];
const string roundTypeInClamp = (string)ConfigParameters["roundTypeInClamp"];
const string runMode = (string)ConfigParameters["runMode"];

const string readTaskMode = (string)ConfigParameters["readTaskMode"];
const int granularityInBF = (int)ConfigParameters["granularityInBF"];
const double eliminateVariableThreshold = (double)ConfigParameters["eliminateVariableThreshold"];
const int debugMode = (int)ConfigParameters["debugMode"];

const double relErrorTolIPM = (double)ConfigParameters["relErrorTolIPM"];
const double weightStep = (double)ConfigParameters["weightStep"];
const double eliminateStep = (double)ConfigParameters["eliminateStep"];
const int iterationNumIPM_Max = (int)ConfigParameters["iterationNumIPM_Max"];
const double xTolIPM = (double)ConfigParameters["xTolIPM"];
