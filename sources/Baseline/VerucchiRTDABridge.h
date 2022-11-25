#ifndef VERUCCHI_RTDA_BRDIGE_H_
#define VERUCCHI_RTDA_BRDIGE_H_
// below are include files required by Verucchi
// #include "sources/Baseline/VerucchiScheduling.h"

// *************************
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/Verucchi20/DAG/DAG.h"
#include "sources/Baseline/Verucchi20/Evaluation/Scheduling.h"
#include "sources/Baseline/Verucchi20/VariableTaskSet/VariableTaskSet.h"
#include "sources/Baseline/Verucchi20/Evaluation/Evaluation.h"
#include "sources/Utils/OptimizeOrderUtils.h"

// #include "sources/Optimization/OptimizeOrder.h"

RegularTaskSystem::TaskSet GetTaskSet(DAG dag);

// defaut proposser number is 1
VectorDynamic GetInitialEstimate(const DAG &dag, int nproc = 1, std::optional<RegularTaskSystem::TaskSetInfoDerived> tasksInfo = std::nullopt);

void GenerateVariableTaskSetFromDAGModel(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    VariableTaskSet &taskSetVeru,
    std::vector<std::shared_ptr<MultiNode>> &tasksVecVeru);

DAG FindTheBestDag(
    const std::vector<DAG> &allDags,
    const std::vector<std::shared_ptr<MultiNode>> &tasksVecVeru,
    const std::vector<std::vector<int>> &causeEffectChains,
    int processorsAvailable = GlobalVariablesDAGOpt::coreNumberAva,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50);

DAG FindTheBestDagWhileCreatingDagsWithTimeLimit(
    VariableTaskSet &taskSetVeru,
    const std::vector<std::shared_ptr<MultiNode>> &tasksVecVeru,
    const std::vector<std::vector<int>> &causeEffectChains,
    int processorsAvailable = GlobalVariablesDAGOpt::coreNumberAva,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50,
    int64_t seconds = GlobalVariablesDAGOpt::kVerucchiTimeLimit);

OrderOptDAG_SPACE::RTDA GetRTDAFromBestDag(
    const DAG &bestDAG,
    RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const std::vector<std::vector<int>> &causeEffectChains,
    int processorsAvailable = GlobalVariablesDAGOpt::coreNumberAva);

// get verucchi's RTDA on multiple chains
OrderOptDAG_SPACE::RTDA GetVerucchiRTDA(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    std::vector<std::vector<int>> causeEffectChains,
    int processorsAvailable = GlobalVariablesDAGOpt::coreNumberAva,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50,
    int64_t time_limit = GlobalVariablesDAGOpt::kVerucchiTimeLimit);

OrderOptDAG_SPACE::ScheduleResult ScheduleVerucchiRTDA(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    std::vector<std::vector<int>> causeEffectChains,
    int processorsAvailable,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50,
    int64_t time_limit = GlobalVariablesDAGOpt::kVerucchiTimeLimit);

#endif // VERUCCHI_RTDA_BRDIGE_H_