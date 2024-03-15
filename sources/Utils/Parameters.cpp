#include "Parameters.h"

#include <yaml-cpp/yaml.h>

namespace GlobalVariablesDAGOpt {
const std::string PROJECT_PATH = std::string(PROJECT_ROOT_DIR) + "/";
YAML::Node loaded_doc = YAML::LoadFile(GlobalVariablesDAGOpt::PROJECT_PATH +
                                       "sources/parameters.yaml");

double barrierBase = loaded_doc["barrierBase"].as<double>();
double coolingRateSA = loaded_doc["coolingRateSA"].as<double>();
double deltaInitialDogleg = loaded_doc["deltaInitialDogleg"].as<double>();
double deltaOptimizer =
    loaded_doc["deltaOptimizer"]
        .as<double>();  // not used in the main SF optimization program
int timeScaleFactor = loaded_doc["timeScaleFactor"].as<int>();
int RepeatExecution = loaded_doc["RepeatExecution"].as<int>();

double initialLambda = loaded_doc["initialLambda"].as<double>();
double lowerLambda = loaded_doc["lowerLambda"].as<double>();
double makespanWeight =
    loaded_doc["makespanWeight"]
        .as<double>();  // not used in the main SF optimization program
double noiseModelSigma =
    loaded_doc["noiseModelSigma"]
        .as<double>();  // not used in the main SF optimization program
double AcceptSchedulError = loaded_doc["AcceptSchedulError"].as<double>();
double parallelFactor = loaded_doc["parallelFactor"].as<double>();
double relativeErrorTolerance =
    loaded_doc["relativeErrorTolerance"].as<double>();

double toleranceEliminator = loaded_doc["toleranceEliminator"].as<double>();
double upperLambda = loaded_doc["upperLambda"].as<double>();
double weightLogBarrier = loaded_doc["weightLogBarrier"].as<double>();
double zeroJacobianDetectTol = loaded_doc["zeroJacobianDetectTol"].as<double>();
double stepJacobianIteration = loaded_doc["stepJacobianIteration"].as<double>();
double NumericalPrecision = loaded_doc["NumericalPrecision"].as<double>();

int printSchedule = loaded_doc["printSchedule"].as<int>();
int bigJobGroupCheck = loaded_doc["bigJobGroupCheck"].as<int>();
int PrintOutput = loaded_doc["PrintOutput"].as<int>();
int ResetInnerWeightLoopMax = loaded_doc["ResetInnerWeightLoopMax"].as<int>();
int coreNumberAva = loaded_doc["coreNumberAva"].as<int>();
int numericalJaobian = loaded_doc["numericalJaobian"].as<int>();
int setUseFixedLambdaFactor = loaded_doc["setUseFixedLambdaFactor"].as<int>();
int setDiagonalDamping = loaded_doc["setDiagonalDamping"].as<int>();
int RandomDrawWeightMaxLoop = loaded_doc["RandomDrawWeightMaxLoop"].as<int>();

// ********************* Under experiment or not useful
int enableFastSearch = loaded_doc["enableFastSearch"].as<int>();
int enableSmallJobGroupCheck = loaded_doc["enableSmallJobGroupCheck"].as<int>();
int subJobGroupMaxSize = loaded_doc["subJobGroupMaxSize"].as<int>();

// ********************* TO-FIX:
double sensorFusionTolerance = loaded_doc["sensorFusionTolerance"].as<double>();
double freshTol = loaded_doc["freshTol"].as<double>();
int initializeMethod = loaded_doc["initializeMethod"].as<int>();
int breakChainThresholdIA = loaded_doc["breakChainThresholdIA"].as<int>();
double activeJobThresholdIA = loaded_doc["activeJobThresholdIA"].as<double>();

double weightDDL_factor =
    loaded_doc["weightDDL_factor"]
        .as<double>();  // not used in the main SF optimization program
double weightDAG_factor =
    loaded_doc["weightDAG_factor"]
        .as<double>();  // not used in the main SF optimization program
double weightInMpRTDA = loaded_doc["weightInMpRTDA"].as<double>();
double weightInMpSf = loaded_doc["weightInMpSf"].as<double>();
double weightInMpSfPunish = loaded_doc["weightInMpSfPunish"].as<double>();
double weightInMpRTDAPunish = loaded_doc["weightInMpRTDAPunish"].as<double>();
double chanceRandomSearch = loaded_doc["chanceRandomSearch"].as<double>();
int64_t OPTIMIZE_TIME_LIMIT = loaded_doc["OPTIMIZE_TIME_LIMIT"].as<int64_t>();
double RtdaWeight = loaded_doc["RtdaWeight"].as<double>();
double ResetRandomWeightThreshold =
    loaded_doc["ResetRandomWeightThreshold"].as<double>();
double weightSF_factor =
    loaded_doc["weightSF_factor"]
        .as<double>();  // not used in the main SF optimization program
int TotalMethodUnderComparison =
    loaded_doc["TotalMethodUnderComparison"].as<int>();
int processorAssignmentMode = loaded_doc["processorAssignmentMode"].as<int>();
int PrintInitial = loaded_doc["PrintInitial"].as<int>();
int debugMode = loaded_doc["debugMode"].as<int>();

int exactJacobian = loaded_doc["exactJacobian"].as<int>();
int optimizerType = loaded_doc["optimizerType"].as<int>();
int randomInitialize = loaded_doc["randomInitialize"].as<int>();
int SA_iteration = loaded_doc["SA_iteration"].as<int>();
int TaskSetType = loaded_doc["TaskSetType"].as<int>();
int temperatureSA = loaded_doc["temperatureSA"].as<int>();
int tightEliminate =
    loaded_doc["tightEliminate"]
        .as<int>();  // not used in the main SF optimization program
int withAddedSensorFusionError =
    loaded_doc["withAddedSensorFusionError"]
        .as<int>();  // not used in the main SF optimization program
int whetherRandomNoiseModelSigma =
    loaded_doc["whetherRandomNoiseModelSigma"]
        .as<int>();  // not used in the main SF optimization program
int whether_ls =
    loaded_doc["whether_ls"]
        .as<int>();  // not used in the main SF optimization program

int whether_shuffle_CE_chain = loaded_doc["whether_shuffle_CE_chain"].as<int>();
int NumCauseEffectChain = loaded_doc["NumCauseEffectChain"].as<int>();
int maxIterations = loaded_doc["maxIterations"].as<int>();
int saveGraph = loaded_doc["saveGraph"].as<int>();
int recordActionValue = loaded_doc["recordActionValue"].as<int>();
int MaxEliminateDetectIterations =
    loaded_doc["MaxEliminateDetectIterations"].as<int>();

std::string priorityMode = loaded_doc["priorityMode"].as<std::string>();
std::string runMode = loaded_doc["runMode"].as<std::string>();
std::string testDataSetName = loaded_doc["testDataSetName"].as<std::string>();
std::string ReOrderProblem = loaded_doc["ReOrderProblem"].as<std::string>();
std::string verbosityLM = loaded_doc["verbosityLM"].as<std::string>();

double punishmentInBarrier = loaded_doc["punishmentInBarrier"].as<double>();
double ReactionTimeThreshold = loaded_doc["ReactionTimeThreshold"].as<double>();
double DataAgeThreshold = loaded_doc["DataAgeThreshold"].as<double>();
double RoundingThreshold = loaded_doc["RoundingThreshold"].as<double>();

double kVerucchiReactionCost = loaded_doc["kVerucchiReactionCost"].as<double>();
double kVerucchiMaxReaction = loaded_doc["kVerucchiMaxReaction"].as<double>();
double kVerucchiMaxDataAge = loaded_doc["kVerucchiMaxDataAge"].as<double>();
double kVerucchiDataAgeCost = loaded_doc["kVerucchiDataAgeCost"].as<double>();
double kVerucchiCoreCost = loaded_doc["kVerucchiCoreCost"].as<double>();
double kGlobalOptimizationTimeLimit =
    loaded_doc["kGlobalOptimizationTimeLimit"].as<double>();

double kCplexInequalityThreshold =
    loaded_doc["kCplexInequalityThreshold"].as<double>();

int selectInitialFromPool = loaded_doc["selectInitialFromPool"].as<int>();
int MakeProgressOnlyMax = loaded_doc["MakeProgressOnlyMax"].as<int>();

double LPTolerance = loaded_doc["LPTolerance"].as<double>();
}  // namespace GlobalVariablesDAGOpt