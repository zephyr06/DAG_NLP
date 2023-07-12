#pragma once
#include <Eigen/Core>
#include <iostream>
#include <vector>
// #include <Eigen/Geometry>
#include "sources/Utils/testMy.h"
// #include <opencv2/core/core.hpp>
// using namespace std;
// All the global variables should be const

enum InitializeMethod {
    IndexMode,
    FixedRelativeStart,
    RM,
    RM_DAG,
    Custom_DAG,
    ListScheduling
};
inline InitializeMethod Transform_enum(int a) {
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
    else {
        CoutError("Not recognized enum InitializeMethod\n");
        return IndexMode;
    }
}

namespace GlobalVariablesDAGOpt {
extern const std::string PROJECT_PATH;

extern double barrierBase;
extern double coolingRateSA;
extern double deltaInitialDogleg;
extern double deltaOptimizer;  // not used in the main SF optimization program
extern int timeScaleFactor;

extern double initialLambda;
extern double lowerLambda;
extern double makespanWeight;   // not used in the main SF optimization program
extern double noiseModelSigma;  // not used in the main SF optimization program
extern double AcceptSchedulError;
extern double parallelFactor;
extern double relativeErrorTolerance;

extern double toleranceEliminator;
extern double upperLambda;
extern double weightLogBarrier;
extern double zeroJacobianDetectTol;
extern double stepJacobianIteration;

extern int printSchedule;
extern int bigJobGroupCheck;
extern int PrintOutput;
extern int ResetInnerWeightLoopMax;
extern int coreNumberAva;
extern int numericalJaobian;  // not used in the main SF optimization program
extern int setUseFixedLambdaFactor;
extern int setDiagonalDamping;
extern int RandomDrawWeightMaxLoop;

// ********************* Under experiment or not useful
extern int enableFastSearch;
extern int enableSmallJobGroupCheck;
extern int RepeatExecution;

extern int subJobGroupMaxSize;

// ********************* TO-FIX:
extern double sensorFusionTolerance;
extern double freshTol;

extern int initializeMethod;
extern double weightDDL_factor;  // not used in the main SF optimization program
extern double weightDAG_factor;  // not used in the main SF optimization program
extern double weightInMpRTDA;
extern double weightInMpSf;
extern double weightInMpSfPunish;
extern double weightInMpRTDAPunish;
extern int64_t OPTIMIZE_TIME_LIMIT;
extern double NumericalPrecision;

extern double RtdaWeight;
extern double ResetRandomWeightThreshold;
extern double weightSF_factor;  // not used in the main SF optimization program
extern int TotalMethodUnderComparison;
extern int processorAssignmentMode;
extern int PrintInitial;
extern int debugMode;
extern int exactJacobian;
extern int optimizerType;
extern int randomInitialize;
extern int SA_iteration;
extern int TaskSetType;
extern int temperatureSA;
extern int tightEliminate;  // not used in the main SF optimization program
extern int
    withAddedSensorFusionError;  // not used in the main SF optimization program
extern int whetherRandomNoiseModelSigma;  // not used in the main SF
                                          // optimization program
extern int whether_ls;  // not used in the main SF optimization program

extern int whether_shuffle_CE_chain;
extern int NumCauseEffectChain;

extern int maxIterations;
extern int saveGraph;
extern int recordActionValue;
extern int MaxEliminateDetectIterations;
extern std::string priorityMode;
extern std::string runMode;
extern std::string testDataSetName;
extern std::string ReOrderProblem;
extern double punishmentInBarrier;
extern double DataAgeThreshold;
extern double ReactionTimeThreshold;
extern double RoundingThreshold;

extern std::string verbosityLM;
extern double kVerucchiReactionCost;
extern double kVerucchiMaxReaction;
extern double kVerucchiDataAgeCost;
extern double kVerucchiMaxDataAge;
extern double kVerucchiCoreCost;
extern double kGlobalOptimizationTimeLimit;

extern double kCplexInequalityThreshold;

extern int selectInitialFromPool;
extern int EnableHardJobORder;
extern double LPTolerance;
extern int MakeProgressOnlyMax;

}  // namespace GlobalVariablesDAGOpt