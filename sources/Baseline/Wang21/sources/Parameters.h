#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "testMy.h"
#include "sources/Utils/Parameters.h"
namespace RTSS21IC_NLP
{
    static std::vector<uint> processorIdVecGlobal; // global variable
    static int processorNumGlobal = 2;             // global variable
    static const std::string PROJECT_PATH_IC = PROJECT_PATH + "sources/Baseline/Wang21/";
    // const std::string PROJECT_PATH = "/home/dong/workspace/DAG_NLP/";
    //  std::string path = PROJECT_PATH + "sources/parameters.yaml";
    static cv::FileStorage ConfigParameters(PROJECT_PATH_IC + "sources/parameters.yaml", cv::FileStorage::READ);
    using namespace std;
    enum InitializeMethod
    {
        IndexMode,
        FixedRelativeStart,
        RM,
        RM_DAG
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
        else
        {
            CoutError("Not recognized enum InitializeMethod\n");
            return IndexMode;
        }
    }
    static const double barrierBase = (double)ConfigParameters["barrierBase"];
    static const double coolingRateSA = (double)ConfigParameters["coolingRateSA"];
    static const double deltaInitialDogleg = (double)ConfigParameters["deltaInitialDogleg"];
    static const double deltaOptimizer = (double)ConfigParameters["deltaOptimizer"];
    static const int timeScaleFactor = (int)ConfigParameters["timeScaleFactor"];

    static const double initialLambda = (double)ConfigParameters["initialLambda"];
    static const double lowerLambda = (double)ConfigParameters["lowerLambda"];
    static double makespanWeight = (double)ConfigParameters["makespanWeight"];
    static const double noiseModelSigma = (double)ConfigParameters["noiseModelSigma"];
    static const double AcceptSchedulError = (double)ConfigParameters["AcceptSchedulError"];
    static const double parallelFactor = (double)ConfigParameters["parallelFactor"];
    static const double relativeErrorTolerance = (double)ConfigParameters["relativeErrorTolerance"];
    static double sensorFusionTolerance = (double)ConfigParameters["sensorFusionTolerance"] * timeScaleFactor;
    static const double toleranceEliminator = (double)ConfigParameters["toleranceEliminator"];
    static const double upperLambda = (double)ConfigParameters["upperLambda"];
    static const double weightLogBarrier = (double)ConfigParameters["weightLogBarrier"];
    static const double zeroJacobianDetectTol = (double)ConfigParameters["zeroJacobianDetectTol"];
    static double freshTol = (double)ConfigParameters["freshTol"];
    static const double stepJacobianIteration = (double)ConfigParameters["stepJacobianIteration"];

    static const int moreElimination = (int)ConfigParameters["moreElimination"];
    static const int ElimnateLoop_Max = (int)ConfigParameters["ElimnateLoop_Max"];

    static int numericalJaobian = (int)ConfigParameters["numericalJaobian"];
    static const int setUseFixedLambdaFactor = (int)ConfigParameters["setUseFixedLambdaFactor"];
    static InitializeMethod initializeMethod = Transform_enum((int)ConfigParameters["initializeMethod"]);
    static const double weightPrior_factor = (double)ConfigParameters["weightPrior_factor"];
    static const double weightDDL_factor = (double)ConfigParameters["weightDDL_factor"];
    static int debugMode = (int)ConfigParameters["debugMode"];
    static const int exactJacobian = (int)ConfigParameters["exactJacobian"];
    static const int batchTestMethod = (int)ConfigParameters["batchTestMethod"];
    static const int optimizerType = (int)ConfigParameters["optimizerType"];
    static const int overlapMode = (int)ConfigParameters["overlapMode"];
    static const int randomInitialize = (int)ConfigParameters["randomInitialize"];
    static const int SA_iteration = (int)ConfigParameters["SA_iteration"];
    static const int TaskSetType = (int)ConfigParameters["TaskSetType"];
    static const int temperatureSA = (int)ConfigParameters["temperatureSA"];
    static int tightEliminate = (int)ConfigParameters["tightEliminate"];
    static int withAddedSensorFusionError = (int)ConfigParameters["withAddedSensorFusionError"];
    static const int maxIterations = (int)ConfigParameters["maxIterations"];
    static const int maxJacobianIteration = (int)ConfigParameters["maxJacobianIteration"];
    static string priorityMode = (std::string)ConfigParameters["priorityMode"];
    static const string readTaskMode = (std::string)ConfigParameters["readTaskMode"];
    static const string runMode = (std::string)ConfigParameters["runMode"];
    static const string testDataSetName = (std::string)ConfigParameters["testDataSetName"];
    static double punishmentInBarrier = (double)ConfigParameters["punishmentInBarrier"];

} // namespace RTSS21IC_NLP