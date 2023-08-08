#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sys/stat.h>
#include <filesystem>
#include "sources/Utils/testMy.h"
#include "sources/Utils/Parameters.h"
namespace RTSS21IC_NLP
{
    extern std::vector<uint> processorIdVecGlobal; // global variable
    extern int processorNumGlobal;                 // global variable

    extern const std::string PROJECT_PATH_IC;

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

    extern double barrierBase;
    extern double coolingRateSA;
    extern double deltaInitialDogleg;
    extern double deltaOptimizer;
    extern int timeScaleFactor;

    extern double initialLambda;
    extern double lowerLambda;
    extern double makespanWeight;
    extern double noiseModelSigma;
    extern double AcceptSchedulError;
    extern double parallelFactor;
    extern double relativeErrorTolerance;

    extern double sensorFusionTolerance;
    extern double toleranceEliminator;
    extern double upperLambda;
    extern double weightLogBarrier;
    extern double zeroJacobianDetectTol;
    extern double freshTol;
    extern double stepJacobianIteration;

    extern int moreElimination;
    extern int ElimnateLoop_Max;
    extern int numericalJaobian;
    extern int setUseFixedLambdaFactor;
    extern int initializeMethod;

    extern double weightDDL_factor;
    extern double weightPrior_factor;

    extern int debugMode;
    extern int exactJacobian;
    extern int batchTestMethod;
    extern int optimizerType;
    extern int overlapMode;
    extern int randomInitialize;
    extern int SA_iteration;
    extern int TaskSetType;
    extern int temperatureSA;
    extern int tightEliminate;
    extern int withAddedSensorFusionError;
    extern int maxIterations;
    extern int maxJacobianIteration;

    extern std::string priorityMode;
    extern std::string readTaskMode;
    extern std::string runMode;
    extern std::string testDataSetName;
    extern double punishmentInBarrier;

} // namespace RTSS21IC_NLP