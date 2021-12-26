
#pragma once
#include "map"
#include "unordered_map"

#include "RegularTasks.h"
#include "DAG_Model.h"
#include "MakeSpanFactor.h"
#include "DAG_ConstraintFactor.h"
#include "DBF_ConstraintFactor.h"
#include "DBF_ConstraintFactor_Multi.h"
#include "DDL_ConstraintFactor.h"
#include "SensorFusionFactor.h"
#include "PriorFactor.h"
#include "EliminationForest_utils.h"
#include "InitialEstimate.h"
#include "colormod.h"

Values GenerateInitialFG(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo)
{
    Values initialEstimateFG;
    Symbol key('a', 0);

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            Symbol key = GenerateKey(index_overall);
            VectorDynamic v = GenerateVectorDynamic(1);
            v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);
            initialEstimateFG.insert(key, v);
        }
    }
    return initialEstimateFG;
}
VectorDynamic CollectUnitOptResult(Values &result, TaskSetInfoDerived &tasksInfo)
{
    VectorDynamic stvAfter = GenerateVectorDynamic(tasksInfo.variableDimension);
    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            Symbol key = GenerateKey(index_overall);
            VectorDynamic aaa = result.at<VectorDynamic>(key);
            stvAfter(index_overall, 0) = result.at<VectorDynamic>(key)(0, 0);
        }
    }
    return stvAfter;
}
using namespace RegularTaskSystem;
// -------------------------------------------------------- from previous optimization begins

// -------------------------------------------------------- from previous optimization ends

namespace DAG_SPACE
{

    LLint CountSFError(DAG_Model &dagTasks, vector<LLint> &sizeOfVariables)
    {
        LLint errorDimensionSF = 0;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            if ((itr->second).size() > 1)
                errorDimensionSF += sizeOfVariables[(itr->first)];
        }
        return errorDimensionSF;
    }

    // void BuildFactorGraph(DAG_Model &dagTasks, NonlinearFactorGraph &graph,
    //                       TaskSetInfoDerived &tasksInfo, EliminationForest &forestInfo)
    // {
    //     TaskSet tasks = dagTasks.tasks;
    //     Symbol key('a', 0);
    //     // LLint errorDimensionMS = 1;

    //     LLint errorDimensionDAG = dagTasks.edgeNumber();
    //     auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    //     graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, tasksInfo, forestInfo,
    //                                                errorDimensionDAG, model);
    //     if (makespanWeight > 0)
    //     {
    //         LLint errorDimensionMS = 1;
    //         model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    //         graph.emplace_shared<MakeSpanFactor>(key, dagTasks, tasksInfo, forestInfo,
    //                                              errorDimensionMS, model);
    //     }

    //     ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    //     LLint errorDimensionDBF = processorTaskSet.size();
    //     model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    //     graph.emplace_shared<DBF_ConstraintFactor>(key, tasksInfo, forestInfo,
    //                                                errorDimensionDBF,
    //                                                model);

    //     LLint errorDimensionDDL = 2 * tasksInfo.variableDimension;
    //     model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    //     graph.emplace_shared<DDL_ConstraintFactor>(key, tasksInfo, forestInfo,
    //                                                errorDimensionDDL, model);

    //     if (weightPrior_factor > 0)
    //     {
    //         LLint errorDimensionPrior = 1;
    //         vector<int> order = FindDependencyOrder(dagTasks);
    //         model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
    //         graph.emplace_shared<Prior_ConstraintFactor>(key, tasksInfo, forestInfo,
    //                                                      errorDimensionPrior, 0.0, order[0], model);
    //     }
    //     // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
    //     // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    //     // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
    //     //                                                     errorDimensionSF, sensorFusionTolerance,
    //     //                                                     mapIndex, maskForEliminate, model);
    // }
    void BuildFactorGraph(DAG_Model &dagTasks, NonlinearFactorGraph &graph,
                          TaskSetInfoDerived &tasksInfo, EliminationForest &forestInfo)
    {
        AddDAG_Factor(graph, dagTasks, tasksInfo);
        // AddDBF_Factor(graph, tasksInfo);
        AddDDL_Factor(graph, tasksInfo);
        // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
        //                                                     errorDimensionSF, sensorFusionTolerance,
        //                                                     mapIndex, maskForEliminate, model);
    }

    double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector)
    {
        NonlinearFactorGraph graph;
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        EliminationForest forestInfo(tasksInfo);
        BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        return graph.error(initialEstimateFG);
    }

    /**
     * @brief Given a example regular task sets, perform instance-level optimization
     * 
     * Example DAG system structure:
        s1 -------- Task0 -------- |
                                   |
        s2 -------- Task1 -------- | -------- Task3 -------- Task4
                                   |
        s3 -------- Task2 -------- |
        
        Event chain: s3 - Task3 - Task4
        Sensor chain: s1 - Task0 - Task3 - Task4

        All tasks are non-preemptive;

        All nodes have the same frequency;

        --------------------------------------------------------------------------------------

        Within this optimizataion problem:
        - variables: start time of all the task instances
        - constraints:
            - DAG dependency
            - self DDL constraints, each instance must complete before the next period begins
            - sensor fusion
            - event chain RTA
     * @return all the instances' start time
     */
    VectorDynamic UnitOptimization(DAG_Model &dagTasks, VectorDynamic &initialEstimate,
                                   EliminationForest &forestInfo,
                                   TaskSetInfoDerived &tasksInfo)
    {
        using namespace RegularTaskSystem;

        NonlinearFactorGraph graph;
        BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);

        Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

        Values result;
        if (optimizerType == 1)
        {
            DoglegParams params;
            if (debugMode >= 1)
                params.setVerbosityDL("VERBOSE");
            params.setDeltaInitial(deltaInitialDogleg);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setMaxIterations(maxIterations);
            DoglegOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 2)
        {
            LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            if (debugMode >= 1)
                params.setVerbosityLM("SUMMARY");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setMaxIterations(maxIterations);
            params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
            // cout << "Log file " << params.getLogFile() << endl;
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = CollectUnitOptResult(result, tasksInfo);
        if (debugMode)
            cout << Color::green << "UnitOptimization finishes for one time" << Color::def << endl;
        return optComp;
    }

    struct OptimizeResult
    {
        double initialError;
        double optimizeError;
        VectorDynamic initialVariable;
        VectorDynamic optimizeVariable;
        OptimizeResult() : initialError(-1), optimizeError(-1)
        {
            ;
        }
        OptimizeResult(double ie, double oe, VectorDynamic iv, VectorDynamic ov) : initialError(ie), optimizeError(oe), initialVariable(iv), optimizeVariable(ov) {}
    };

    /**
     * @brief this function schedules task sets based on initial estimate, 
     * as a baseline evaluation
     * 
     * @param dagTasks 
     * @return OptimizeResult 
     */
    OptimizeResult InitialScheduling(DAG_Model &dagTasks)
    {
        TaskSet tasks = dagTasks.tasks;
        int N = tasks.size();
        LLint hyperPeriod = HyperPeriod(tasks);

        // declare variables
        vector<LLint> sizeOfVariables;
        int variableDimension = 0;
        for (int i = 0; i < N; i++)
        {
            LLint size = hyperPeriod / tasks[i].period;
            sizeOfVariables.push_back(size);
            variableDimension += size;
        }

        VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                        sizeOfVariables, variableDimension);
        double errorInitial = GraphErrorEvaluation(dagTasks, initialEstimate);
        return {errorInitial, errorInitial, initialEstimate, initialEstimate};
    }

    VectorDynamic UpdateInitialVector(VectorDynamic &startTimeComplete,
                                      TaskSetInfoDerived &tasksInfo,
                                      EliminationForest &forestInfo)
    {
        VectorDynamic initialUpdate;
        initialUpdate.resize(forestInfo.lengthCompressed, 1);

        LLint index = 0;
        for (size_t i = 0; i < (size_t)tasksInfo.variableDimension; i++)
        {
            if (not forestInfo.maskForEliminate[i])
            {
                initialUpdate(index++, 0) = startTimeComplete(i, 0);
            }
        }

        return initialUpdate;
    }

    /**
     * @brief Perform scheduling based on optimization
     * 
     * @param tasks 
     * @return VectorDynamic all the task instances' start time
     */
    OptimizeResult OptimizeScheduling(DAG_Model &dagTasks, VectorDynamic initialUser = GenerateVectorDynamic(1))
    {
        TaskSet tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        EliminationForest forestInfo(tasksInfo);

        VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);

        // build elimination eliminationTrees
        bool whetherEliminate = false;

        int loopNumber = 0;
        VectorDynamic resTemp = GenerateVectorDynamic(tasksInfo.variableDimension);
        VectorDynamic trueResult;
        while (1)
        {
            whetherEliminate = false;
            BeginTimer("UnitOptimization");
            resTemp = UnitOptimization(dagTasks, initialEstimate,
                                       forestInfo,
                                       tasksInfo);
            EndTimer("UnitOptimization");
            VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, forestInfo);

            // startTimeComplete = RandomWalk(startTimeComplete, tasksInfo, forestInfo);
            // factors that require elimination analysis are: DBF
            // LLint errorDimensionDBF = tasksInfo.processorTaskSet.size();
            // auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            // Symbol key('b', 0);
            // DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF, model);
            // this function performs in-place modification for all the variables!
            // TODO: should we add eliminate function for sensorFusion?
            // factor.addMappingFunction(resTemp, whetherEliminate, forestInfo);

            if (not whetherEliminate)
            {
                trueResult = startTimeComplete;
                break;
            }
            else
            {
                initialEstimate = UpdateInitialVector(startTimeComplete, tasksInfo, forestInfo);
            }
            loopNumber++;
            if (loopNumber > ElimnateLoop_Max)
            {
                CoutWarning("Loop number Warning in OptimizeScheduling");
                // cannot use mapIndex to recover, because mapIndex has already been changed at this point
                trueResult = startTimeComplete;
                break;
            }
        }

        initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
        double errorInitial = GraphErrorEvaluation(dagTasks, initialEstimate);

        cout << Color::blue << "The error before optimization is "
             << errorInitial << Color::def << endl;
        double finalError = GraphErrorEvaluation(dagTasks, trueResult);
        cout << Color::blue << "The error after optimization is " << finalError << Color::def << endl;
        return {errorInitial, finalError, initialEstimate, trueResult};
    }
}