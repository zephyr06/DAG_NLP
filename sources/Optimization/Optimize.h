
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
        int whetherRandomNoiseModelSigmaRef = whetherRandomNoiseModelSigma;
        whetherRandomNoiseModelSigma = 0;
        NonlinearFactorGraph graph;
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        EliminationForest forestInfo(tasksInfo);
        BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        if (printDetail)
        {
            whetherRandomNoiseModelSigma = whetherRandomNoiseModelSigmaRef;
            std::cout << Color::green;
            auto sth = graph.linearize(initialEstimateFG)->jacobian();
            MatrixDynamic jacobianCurr = sth.first;
            std::cout << "Current Jacobian matrix:" << std::endl;
            std::cout << jacobianCurr << std::endl;
            std::cout << "Current b vector: " << std::endl;
            std::cout << sth.second << std::endl;
            std::cout << Color::def << std::endl;
            whetherRandomNoiseModelSigma = 0;
        }

        double err = graph.error(initialEstimateFG);
        whetherRandomNoiseModelSigma = whetherRandomNoiseModelSigmaRef;
        return err;
    }

    Values SolveFactorGraph(NonlinearFactorGraph &graph, Values &initialEstimateFG)
    {
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
        return result;
    }
    bool ExistVanishGradient(MatrixDynamic &J, VectorDynamic &b)
    {
        LLint m = J.rows();
        LLint n = J.cols();
        for (size_t i = 0; i < m; i++)
        {
            if (b(i) != 0 && J.block(i, 0, 1, n).norm() < zeroJacobianDetectTol * n)
            {
                return true;
            }
        }
        return false;
    }

    void ResetSRand(size_t srandRef)
    {
        srand(srandRef);
        if (debugMode == 1)
        {
            std::cout << "Reset the weight files" << std::endl;
        }
    }

    bool ResetRandomWeightInFG(DAG_Model &dagTasks, NonlinearFactorGraph &graph, VectorDynamic startTimeVector, int srandRef)
    {
        // TODO: probably move this function to unit optimization so that it detects the reset condition using the same Jacobian as the last iteration of optmization process
        if (!(whetherRandomNoiseModelSigma))
            return false;
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);

        if (graph.error(initialEstimateFG) < 1e-2)
        {
            return false;
        }
        // verify whether it contains small $J^T \cdot b$
        // TODO: replace J with sparse matrix type
        // TODO: move this detection outside?
        MatrixDynamic J;
        VectorDynamic b;
        std::tie(J, b) = graph.linearize(initialEstimateFG)->jacobian();
        LLint m = J.rows();
        LLint n = J.cols();
        if (J.norm() < zeroJacobianDetectTol * n) // stationary point
        {
            return false;
        }
        else if (ExistVanishGradient(J, b)) // gradient vanish point
        {
            return false;
        }
        else
        {
            VectorDynamic Jb = J.transpose() * b;
            for (size_t i = 0; i < n; i++)
            {
                if (Jb(i) < ResetRandomWeightThreshold && b(i) > ResetRandomWeightThreshold)
                {
                    ResetSRand(srandRef);
                    return true;
                }
            }
            return false;
        }
        ResetSRand(srandRef);
        return true;
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
        BeginTimer("UnitOptimization");
        int loopNumber = 0;
        VectorDynamic optComp;
        while (loopNumber < RandomDrawWeightMaxLoop)
        {
            Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

            NonlinearFactorGraph graph;
            Values result;
            BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
            result = SolveFactorGraph(graph, initialEstimateFG);

            optComp = CollectUnitOptResult(result, tasksInfo);
            if (debugMode)
                cout << Color::green << "UnitOptimization finishes for one time" << Color::def << endl;
            bool whetherReset = ResetRandomWeightInFG(dagTasks, graph, optComp, ++loopNumber);

            if (!whetherReset)
            {
                break;
            }
        }
        if (loopNumber == RandomDrawWeightMaxLoop)
        {
            CoutWarning("After resetting the random weight parameters for " + std::to_string(RandomDrawWeightMaxLoop) + " times, the Jb norm is still very small!");
        }
        EndTimer("UnitOptimization");
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
    // TODO:  first try begin and end, if both fails, they randomly generate a start time within the feasible region, and keep excluding regions where gradient vanish is possible. Algorithm terminates if there is no such possible space to generate start time.
    enum RelocationMethod
    {
        EndOfInterval,
        BeginOfInterval,
        RandomOfInterval
    };
    inline RelocationMethod IncrementRelocationMethod(RelocationMethod x)
    {
        if (x != RandomOfInterval)
            return static_cast<RelocationMethod>(static_cast<int>(x) + 1);
        else
        {
            return RandomOfInterval;
        }
    }
    VectorDynamic FindEmptyPosition(TaskSetInfoDerived &tasksInfo, gtsam::Symbol smallJobKey, gtsam::Symbol largeJobKey, VectorDynamic &startTimeVector, RelocationMethod relocateMethod = EndOfInterval)
    {
        VectorDynamic stvRes = startTimeVector;
        int smallTaskIndex, smallJobIndex, largeTaskIndex, largeJobIndex;
        std::tie(smallTaskIndex, smallJobIndex) = AnalyzeKey(smallJobKey);
        std::tie(largeTaskIndex, largeJobIndex) = AnalyzeKey(largeJobKey);
        LLint indexSmallInSTV = IndexTran_Instance2Overall(smallTaskIndex, smallJobIndex, tasksInfo.sizeOfVariables);
        LLint indexLargeInSTV = IndexTran_Instance2Overall(largeTaskIndex, largeJobIndex, tasksInfo.sizeOfVariables);
        switch (relocateMethod)
        {
        case BeginOfInterval:
            // put it at the begining of large task
            stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) - tasksInfo.tasks[smallTaskIndex].executionTime;
            break;
        case EndOfInterval:
            // put it at the end of large task
            stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) + tasksInfo.tasks[largeTaskIndex].executionTime;
            break;
        case RandomOfInterval:
            stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) - tasksInfo.tasks[smallTaskIndex].executionTime;
            break;
        }

        return stvRes;
    }

    struct GradientVanishPairs
    {
        std::vector<std::pair<gtsam::Symbol, gtsam::Symbol>> vanishPairs_;
        void add(gtsam::Symbol key1, gtsam::Symbol key2)
        {
            vanishPairs_.push_back(std::make_pair(key1, key2));
        }
        size_t size() const
        {
            return vanishPairs_.size();
        }
    };
    bool operator==(const GradientVanishPairs &p1, const GradientVanishPairs &p2)
    {
        if (p1.size() != p2.size())
        {
            return false;
        }
        else
        {
            for (size_t i = 0; i < p1.size(); i++)
            {
                if (p1.vanishPairs_[i].first != p2.vanishPairs_[i].first || p1.vanishPairs_[i].second != p2.vanishPairs_[i].second)
                {
                    return false;
                }
            }
        }
        return true;
    }
    // move some variables that suffer from zero gradient issue around
    std::pair<bool, VectorDynamic> RelocateIncludedInterval(TaskSetInfoDerived &tasksInfo,
                                                            DAG_Model &dagTasks, EliminationForest &forestInfo,
                                                            VectorDynamic &startTimeVector, RelocationMethod relocateMethod = EndOfInterval)
    {
        NonlinearFactorGraph graph;
        BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);

        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        bool whetherRelocate = false;
        for (auto ite = graph.begin(); ite != graph.end(); ite++)
        {
            double err = ite->get()->error(initialEstimateFG);
            if (err == 0)
            {
                continue;
            }
            double deltaOptimizerRef = deltaOptimizer;
            deltaOptimizer = 1e-6;
            auto pp = ite->get()->linearize(initialEstimateFG)->jacobian();
            deltaOptimizer = deltaOptimizerRef;
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
                    startTimeVector = FindEmptyPosition(tasksInfo, keys[0], keys[1], startTimeVector, relocateMethod);

                    initialEstimateFG.update(keys[0], GenerateVectorDynamic1D(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, task0Index, job0Index)));
                }
                else
                {
                    startTimeVector = FindEmptyPosition(tasksInfo, keys[1], keys[0], startTimeVector, relocateMethod);
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

        // build elimination eliminationTrees
        bool whetherEliminate = false;

        int loopNumber = 0;
        VectorDynamic resTemp = GenerateVectorDynamic(tasksInfo.variableDimension);
        VectorDynamic trueResult;
        double prevError = INT32_MAX;
        // this makes sure we get the same result every time we run the program
        srand(ElimnateLoop_Max + 1);
        RelocationMethod currentRelocationMethod = EndOfInterval;
        while (loopNumber < ElimnateLoop_Max)
        {
            whetherEliminate = false;

            resTemp = UnitOptimization(dagTasks, initialEstimate,
                                       forestInfo,
                                       tasksInfo);

            VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, forestInfo);

            // convergence check, prevent dead-end loops, update trueResult
            double currError = GraphErrorEvaluation(dagTasks, startTimeComplete);
            if (currError < 1e-3)
            {
                trueResult = startTimeComplete;
                break;
            }
            else if (currError < prevError)
            {
                trueResult = startTimeComplete;
                prevError = currError;
            }
            else
            {
                CoutWarning("Error increased!");
                // break;
            }

            if (std::abs(currError - prevError) / prevError < relativeErrorTolerance)
            {
                currentRelocationMethod = IncrementRelocationMethod(currentRelocationMethod);
            }

            bool whetherChanged = false;
            std::tie(whetherChanged, startTimeComplete) = RelocateIncludedInterval(tasksInfo, dagTasks, forestInfo, startTimeComplete, currentRelocationMethod);
            if (whetherChanged)
            {
                initialEstimate = UpdateInitialVector(startTimeComplete, tasksInfo, forestInfo);
            }

            if (!whetherChanged)
            {
                break;
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
            if (loopNumber >= ElimnateLoop_Max)
                CoutWarning("Loop number Warning in OptimizeScheduling");
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