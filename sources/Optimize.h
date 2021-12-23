
#pragma once
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

    double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector)
    {
        TaskSet tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        EliminationForest forestInfo(tasksInfo);
        Symbol key('a', 0);
        NonlinearFactorGraph graph;
        // LLint errorDimensionMS = 1;

        LLint errorDimensionDAG = dagTasks.edgeNumber();
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, tasksInfo, forestInfo,
                                                   errorDimensionDAG, model);
        if (makespanWeight > 0)
        {
            LLint errorDimensionMS = 1;
            model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
            graph.emplace_shared<MakeSpanFactor>(key, dagTasks, tasksInfo, forestInfo,
                                                 errorDimensionMS, model);
        }

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
        LLint errorDimensionDBF = processorTaskSet.size();
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                   errorDimensionDBF,
                                                   model);

        LLint errorDimensionDDL = 2 * tasksInfo.variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                   errorDimensionDDL, model);

        if (weightPrior_factor > 0)
        {
            LLint errorDimensionPrior = 1;
            vector<int> order = FindDependencyOrder(dagTasks);
            model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
            graph.emplace_shared<Prior_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                         errorDimensionPrior, 0.0, order[0], model);
        }

        // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
        //                                                     errorDimensionSF, sensorFusionTolerance,
        //                                                     mapIndex, maskForEliminate, model);
        Values initialEstimateFG;
        // Symbol key('a', 0);
        initialEstimateFG.insert(key, startTimeVector);
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
    //    vector<LLint> &sizeOfVariables, int variableDimension,
    //    LLint hyperPeriod)
    {
        using namespace RegularTaskSystem;

        // int N = dagTasks.tasks.size();

        // build the factor graph
        NonlinearFactorGraph graph;
        Symbol key('a', 0);

        LLint errorDimensionDAG = dagTasks.edgeNumber();
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, tasksInfo, forestInfo,
                                                   errorDimensionDAG, model);
        if (makespanWeight > 0)
        {
            LLint errorDimensionMS = 1;
            model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
            graph.emplace_shared<MakeSpanFactor>(key, dagTasks, tasksInfo, forestInfo,
                                                 errorDimensionMS, model);
        }

        ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
        LLint errorDimensionDBF = processorTaskSet.size();
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                   errorDimensionDBF,
                                                   model);

        LLint errorDimensionDDL = 2 * tasksInfo.variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                   errorDimensionDDL, model);

        if (weightPrior_factor > 0)
        {
            LLint errorDimensionPrior = 1;
            vector<int> order = FindDependencyOrder(dagTasks);
            model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
            graph.emplace_shared<Prior_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                         errorDimensionPrior, 0.0, order[0], model);
        }

        // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
        //                                                     errorDimensionSF, sensorFusionTolerance,
        //                                                     mapIndex, maskForEliminate, model);
        // return graph;

        Values initialEstimateFG;
        initialEstimateFG.insert(key, initialEstimate);

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
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
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

    VectorDynamic RandomWalk(VectorDynamic &startTimeVector, DAG_Model &dagTasks,
                             EliminationForest &forestInfo)
    {
        // IntervalTree={}
        // for each interval intV:
        //     if interval
        return startTimeVector;
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

            startTimeComplete = RandomWalk(startTimeComplete, dagTasks, forestInfo);
            // factors that require elimination analysis are: DBF
            // ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
            LLint errorDimensionDBF = tasksInfo.processorTaskSet.size();

            auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            Symbol key('b', 0);
            DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF, model);
            // this function performs in-place modification for all the variables!
            // TODO: should we add eliminate function for sensorFusion?
            factor.addMappingFunction(resTemp, whetherEliminate, forestInfo);

            // update initial estimate

            initialEstimate = UpdateInitialVector(startTimeComplete, tasksInfo, forestInfo);

            if (not whetherEliminate)
            {
                trueResult = RecoverStartTimeVector(resTemp, forestInfo);
                break;
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