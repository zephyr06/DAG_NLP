
#include "sources/Baseline/Wang21/sources/Parameters.h"
namespace RTSS21IC_NLP
{
    std::vector<uint> processorIdVecGlobal; // global variable
    int processorNumGlobal = 2;             // global variable
    // std::cout << GlobalVariablesDAGOpt::PROJECT_PATH << "\n";
    const std::string PROJECT_PATH_IC = std::string(PROJECT_ROOT_DIR) + "/" +
                                        "sources/Baseline/Wang21/";

    // AssertFileExist(yaml_file_path);
    YAML::Node loaded_doc_ic = YAML::LoadFile(PROJECT_PATH_IC +
                                              "sources/parameters.yaml");

    double barrierBase = loaded_doc_ic["barrierBase"].as<double>();
    double coolingRateSA = loaded_doc_ic["coolingRateSA"].as<double>();
    double deltaInitialDogleg = loaded_doc_ic["deltaInitialDogleg"].as<double>();
    double deltaOptimizer = loaded_doc_ic["deltaOptimizer"].as<double>(); // not used in the main SF optimization program
    int timeScaleFactor = loaded_doc_ic["timeScaleFactor"].as<int>();

    double initialLambda = loaded_doc_ic["initialLambda"].as<double>();
    double lowerLambda = loaded_doc_ic["lowerLambda"].as<double>();
    double makespanWeight = loaded_doc_ic["makespanWeight"].as<double>();   // not used in the main SF optimization program
    double noiseModelSigma = loaded_doc_ic["noiseModelSigma"].as<double>(); // not used in the main SF optimization program
    double AcceptSchedulError = loaded_doc_ic["AcceptSchedulError"].as<double>();
    double parallelFactor = loaded_doc_ic["parallelFactor"].as<double>();
    double relativeErrorTolerance = loaded_doc_ic["relativeErrorTolerance"].as<double>();

    double sensorFusionTolerance = loaded_doc_ic["sensorFusionTolerance"].as<double>();
    double toleranceEliminator = loaded_doc_ic["toleranceEliminator"].as<double>();
    double upperLambda = loaded_doc_ic["upperLambda"].as<double>();
    double weightLogBarrier = loaded_doc_ic["weightLogBarrier"].as<double>();
    double zeroJacobianDetectTol = loaded_doc_ic["zeroJacobianDetectTol"].as<double>();
    double freshTol = loaded_doc_ic["freshTol"].as<double>();
    double stepJacobianIteration = loaded_doc_ic["stepJacobianIteration"].as<double>();

    int moreElimination = loaded_doc_ic["moreElimination"].as<int>();
    int ElimnateLoop_Max = loaded_doc_ic["ElimnateLoop_Max"].as<int>();
    int numericalJaobian = loaded_doc_ic["numericalJaobian"].as<int>();
    int setUseFixedLambdaFactor = loaded_doc_ic["setUseFixedLambdaFactor"].as<int>();
    int initializeMethod = loaded_doc_ic["initializeMethod"].as<int>();

    double weightDDL_factor = loaded_doc_ic["weightDDL_factor"].as<double>();
    double weightPrior_factor = loaded_doc_ic["weightPrior_factor"].as<double>();

    int debugMode = loaded_doc_ic["debugMode"].as<int>();
    int exactJacobian = loaded_doc_ic["exactJacobian"].as<int>();
    int batchTestMethod = loaded_doc_ic["batchTestMethod"].as<int>();
    int optimizerType = loaded_doc_ic["optimizerType"].as<int>();
    int overlapMode = loaded_doc_ic["overlapMode"].as<int>();
    int randomInitialize = loaded_doc_ic["randomInitialize"].as<int>();
    int SA_iteration = loaded_doc_ic["SA_iteration"].as<int>();
    int TaskSetType = loaded_doc_ic["TaskSetType"].as<int>();
    int temperatureSA = loaded_doc_ic["temperatureSA"].as<int>();
    int tightEliminate = loaded_doc_ic["tightEliminate"].as<int>();
    int withAddedSensorFusionError = loaded_doc_ic["withAddedSensorFusionError"].as<int>();
    int maxIterations = loaded_doc_ic["maxIterations"].as<int>();
    int maxJacobianIteration = loaded_doc_ic["maxJacobianIteration"].as<int>();

    std::string priorityMode = loaded_doc_ic["priorityMode"].as<std::string>();
    std::string readTaskMode = loaded_doc_ic["readTaskMode"].as<std::string>();
    std::string runMode = loaded_doc_ic["runMode"].as<std::string>();
    std::string testDataSetName = loaded_doc_ic["testDataSetName"].as<std::string>();
    double punishmentInBarrier = loaded_doc_ic["punishmentInBarrier"].as<double>();

} // namespace RTSS21IC_NLP