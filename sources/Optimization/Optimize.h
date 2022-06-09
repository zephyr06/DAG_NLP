
#pragma once
#include "map"
#include "unordered_map"

#include "sources/TaskModel/RegularTasks.h"
#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Factors/DBF_ConstraintFactorNonPreemptive_Multi.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/MakeSpanFactor.h"
#include "sources/Factors/DAG_ConstraintFactor.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Tools/colormod.h"

Values GenerateInitialFG(VectorDynamic &startTimeVector, TaskSetInfoDerived &tasksInfo, bool ifPreemptive = false)
{
    Values initialEstimateFG;
    Symbol key('a', 0); // just declare the variable

    for (int i = 0; i < tasksInfo.N; i++)
    {
        for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
        {
            // LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
            Symbol key = GenerateKey(i, j);
            VectorDynamic v;
            if (ifPreemptive)
            {
                v = GenerateVectorDynamic(2);
                v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j),
                    ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) + tasksInfo.tasks[i].executionTime;
            }
            else
            {
                v = GenerateVectorDynamic(1);
                v << ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j);
            }

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
            Symbol key = GenerateKey(i, j);
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

    void BuildFactorGraph(DAG_Model &dagTasks, NonlinearFactorGraph &graph,
                          TaskSetInfoDerived &tasksInfo, EliminationForest &forestInfo)
    {
        AddDAG_Factor(graph, dagTasks, tasksInfo);
        AddDBF_Factor(graph, tasksInfo);
        AddDDL_Factor(graph, tasksInfo);
        // AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);
        // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
        //                                                     errorDimensionSF, sensorFusionTolerance,
        //                                                     mapIndex, maskForEliminate, model);
    }

    double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector, bool printDetail = false)
    {
        NonlinearFactorGraph graph;
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        EliminationForest forestInfo(tasksInfo);
        BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        if (printDetail)
        {

            std::cout << Color::green;
            auto sth = graph.linearize(initialEstimateFG)->jacobian();
            MatrixDynamic jacobianCurr = sth.first;
            std::cout << "Current Jacobian matrix:" << std::endl;
            std::cout << jacobianCurr << std::endl;
            std::cout << "Current b vector: " << std::endl;
            std::cout << sth.second << std::endl;
            std::cout << Color::def << std::endl;
        }
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
            // if (debugMode >= 1)
            params.setVerbosityLM(verbosityLM);
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setMaxIterations(maxIterations);
            params.setDiagonalDamping(setDiagonalDamping);
            params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
            // cout << "Log file " << params.getLogFile() << endl;
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = CollectUnitOptResult(result, tasksInfo);
        if (saveGraph == 1)
        {
            std::ofstream os("graph.dot");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            graph.saveGraph(os, result);
#pragma GCC diagnostic pop
            // graph.print();
        }

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

    // move start time of small interval to the end of large interval
    VectorDynamic FindEmptyPosition(TaskSetInfoDerived &tasksInfo, gtsam::Symbol smallJobKey, gtsam::Symbol largeJobKey, VectorDynamic &startTimeVector)
    {
        VectorDynamic stvRes = startTimeVector;
        int smallTaskIndex, smallJobIndex, largeTaskIndex, largeJobIndex;
        std::tie(smallTaskIndex, smallJobIndex) = AnalyzeKey(smallJobKey);
        std::tie(largeTaskIndex, largeJobIndex) = AnalyzeKey(largeJobKey);
        LLint indexSmallInSTV = IndexTran_Instance2Overall(smallTaskIndex, smallJobIndex, tasksInfo.sizeOfVariables);
        LLint indexLargeInSTV = IndexTran_Instance2Overall(largeTaskIndex, largeJobIndex, tasksInfo.sizeOfVariables);
        // put it at the end of large task
        stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) + tasksInfo.tasks[largeTaskIndex].executionTime;
        // put it at the begining of large task
        // stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) - tasksInfo.tasks[smallTaskIndex].executionTime;
        return stvRes;
    }

    // move some variables that suffer from zero gradient issue around
    std::pair<bool, VectorDynamic> RelocateIncludedInterval(NonlinearFactorGraph &graph,
                                                            TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector)
    {
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        bool whetherRelocate = false;
        for (auto ite = graph.begin(); ite != graph.end(); ite++)
        {
            double err = ite->get()->error(initialEstimateFG);
            if (err == 0)
            {
                continue;
            }
            auto pp = ite->get()->linearize(initialEstimateFG)->jacobian();
            MatrixDynamic jacobian = pp.first;
            if (jacobian.norm() / err < zeroJacobianDetectTol)
            {
                std::cout << "Vanish DBF factor: " << std::endl;
                ite->get()->printKeys();
                auto keys = ite->get()->keys();
                int task0Index, job0Index, task1Index, job1Index;
                std::tie(task0Index, job0Index) = AnalyzeKey(keys[0]);
                std::tie(task1Index, job1Index) = AnalyzeKey(keys[1]);
                if (tasksInfo.tasks[task0Index].executionTime < tasksInfo.tasks[task1Index].executionTime)
                {
                    startTimeVector = FindEmptyPosition(tasksInfo, keys[0], keys[1], startTimeVector);

                    initialEstimateFG.update(keys[0], GenerateVectorDynamic1D(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, task0Index, job0Index)));
                }
                else
                {
                    startTimeVector = FindEmptyPosition(tasksInfo, keys[1], keys[0], startTimeVector);
                    initialEstimateFG.update(keys[1], GenerateVectorDynamic1D(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, task1Index, job1Index)));
                }
                whetherRelocate = true;
            }
        }
        return std::make_pair(whetherRelocate, startTimeVector);
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
        // initialEstimate << 180, 190, 200, 210, 220;
        // build elimination eliminationTrees
        bool whetherEliminate = false;

        int loopNumber = 0;
        VectorDynamic resTemp = GenerateVectorDynamic(tasksInfo.variableDimension);
        VectorDynamic trueResult;
        double prevError = INT32_MAX;
        while (1)
        {
            whetherEliminate = false;
            BeginTimer("UnitOptimization");
            resTemp = UnitOptimization(dagTasks, initialEstimate,
                                       forestInfo,
                                       tasksInfo);
            EndTimer("UnitOptimization");
            VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, forestInfo);
            trueResult = startTimeComplete;
            // convergence check, prevent dead-end loops
            double currError = GraphErrorEvaluation(dagTasks, startTimeComplete);
            if (currError < 1e-4) // already find global optimal point
            {
                break;
            }
            else if (currError < prevError)
            {
                prevError = currError;
            }
            else if (currError >= prevError)
            {
                loopNumber += int(ElimnateLoop_Max * 0.4);
            }

            // relocate interval with vanish gradient
            NonlinearFactorGraph graph;
            BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
            bool whetherChanged = false;

            std::tie(whetherChanged, startTimeComplete) = RelocateIncludedInterval(graph, tasksInfo, startTimeComplete);
            if (!whetherChanged)
            {
                trueResult = startTimeComplete;
                // try variable elimination
                // ...
                if (not whetherEliminate)
                {
                    trueResult = startTimeComplete;
                    break;
                }
            }
            else
            {
                initialEstimate = UpdateInitialVector(startTimeComplete, tasksInfo, forestInfo);
            }
            /**

            // factors that require elimination analysis are: DBF
            LLint errorDimensionDBF = tasksInfo.processorTaskSet.size();
            auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            Symbol key('b', 0);
            DBF_ConstraintFactor factor(key, tasksInfo, forestInfo, errorDimensionDBF, model);
            // this function performs in-place modification for all the variables!
            // TODO: should we add eliminate function for sensorFusion?
            factor.addMappingFunction(resTemp, whetherEliminate, forestInfo);
            **/

            loopNumber++;
            if (loopNumber > ElimnateLoop_Max)
            {
                CoutWarning("Loop number Warning in OptimizeScheduling");
                // cannot use mapIndex to recover, because mapIndex has already been changed at this point
                // trueResult = startTimeComplete;
                break;
            }
        }

        initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
        double errorInitial = GraphErrorEvaluation(dagTasks, initialEstimate);

        cout << Color::blue << "The error before optimization is "
             << errorInitial << Color::def << endl;
        double finalError = GraphErrorEvaluation(dagTasks, trueResult, debugMode > 0);
        cout << Color::blue << "The error after optimization is " << finalError << Color::def << endl;
        return {errorInitial, finalError, initialEstimate, trueResult};
    }
}