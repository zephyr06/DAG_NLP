
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
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/RelocateStartTimeVector.h"
#include "sources/Optimization/JobGroups.h"
#include "sources/Tools/colormod.h"

using namespace RegularTaskSystem;

// -------------------------------------------------------- from previous optimization begins

// -------------------------------------------------------- from previous optimization ends

namespace OrderOptDAG_SPACE
{

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
    void BuildFactorGraph(DAG_Model &dagTasks, NonlinearFactorGraph &graph,
                          TaskSetInfoDerived &tasksInfo)
    {
        if (weightDAG_factor != 0)
            AddDAG_Factor(graph, dagTasks, tasksInfo);

        AddDBF_Factor(graph, tasksInfo);

        if (weightDDL_factor != 0)
            AddDDL_Factor(graph, tasksInfo);
        if (RtdaWeight != 0)
        {
            if (dagTasks.chains_.size() > 0)
                AddWholeRTDAFactor(graph, tasksInfo, dagTasks.chains_[0]);
        }

        // AddMakeSpanFactor(graph, tasksInfo, dagTasks.mapPrev);
        // LLint errorDimensionSF = CountSFError(dagTasks, tasksInfo.sizeOfVariables);
        // auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
        //                                                     errorDimensionSF, sensorFusionTolerance,
        //                                                     mapIndex, maskForEliminate, model);
        if (weightSF_factor != 0)
            AddSF_Factor(graph, dagTasks, tasksInfo);
    }

    double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector, bool printDetail = false)
    {
        int whetherRandomNoiseModelSigmaRef = whetherRandomNoiseModelSigma;
        whetherRandomNoiseModelSigma = 0;
        NonlinearFactorGraph graph;
        TaskSetInfoDerived tasksInfo(dagTasks.tasks);
        // EliminationForest forestInfo(tasksInfo);
        BuildFactorGraph(dagTasks, graph, tasksInfo);
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
        for (LLint i = 0; i < m; i++)
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
        // LLint m = J.rows();
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
            for (LLint i = 0; i < n; i++)
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
    // VectorDynamic UnitOptimization(DAG_Model &dagTasks, VectorDynamic &initialEstimate,
    //     EliminationForest &forestInfo,
    //     TaskSetInfoDerived &tasksInfo)
    // {
    //     BeginTimer("UnitOptimization");
    //     Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    //     NonlinearFactorGraph graph;
    //     BuildFactorGraph(dagTasks, graph, tasksInfo, forestInfo);
    //     Values result = SolveFactorGraph(graph, initialEstimateFG);
    //     VectorDynamic optComp = CollectUnitOptResult(result, tasksInfo);
    //     if (debugMode)
    //         cout << Color::green << "UnitOptimization finishes for one time" << Color::def << endl;
    //     EndTimer("UnitOptimization");
    //     if (debugMode == 1)
    //     {
    //         auto rtdaVec = GetRTDAFromSingleJob(tasksInfo, { 0,1,4,5 }, result);
    //         RTDA resAfterOpt = GetMaxRTDA(rtdaVec);
    //         std::cout << Color::green << std::endl;
    //         resAfterOpt.print();
    //         std::cout << Color::def << std::endl;
    //     }
    //     return optComp;
    // }

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

    VectorDynamic UpdateInitialVector(VectorDynamic &startTimeComplete,
                                      TaskSetInfoDerived &tasksInfo,
                                      EliminationForest &forestInfo)
    {
        return startTimeComplete;

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

    struct StateActionCollection
    {
        double currError;
        bool whetherVanishGradient;
        int resetWeightSeed;
        int initialOption;
        RelocationMethod relocateMethod;

        StateActionCollection() : currError(1e60), whetherVanishGradient(false), resetWeightSeed(-1), initialOption(-1), relocateMethod(EndOfLongInterval) {}

        inline std::string to_string()
        {
            return ResizeStr(std::to_string(currError)) + ResizeStr(std::to_string(whetherVanishGradient)) + ResizeStr(std::to_string(resetWeightSeed)) + ResizeStr(std::to_string(initialOption)) + ResizeStr(std::to_string(relocateMethod)) + "\n";
        }
    };

    void PrintResultAnalyzation(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, VectorDynamic &x)
    {
        std::cout << "Current start time vector: " << x << std::endl;
        std::cout << Color::blue;
        Values xValues = GenerateInitialFG(x, tasksInfo);
        std::cout << "Overall graph error is: " << GraphErrorEvaluation(dagTasks, x, debugMode == 1) << std::endl;

        NonlinearFactorGraph graph1;
        AddDBF_Factor(graph1, tasksInfo);
        std::cout << "DBF error: " << graph1.error(xValues) << std::endl;

        NonlinearFactorGraph graph2;
        AddDDL_Factor(graph2, tasksInfo);
        std::cout << "DDL error: " << graph2.error(xValues) << std::endl;

        // RTDA
        if (dagTasks.chains_.size() > 0)
        {
            auto rtdaVec = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], xValues);
            RTDA resAfterOpt = GetMaxRTDA(rtdaVec);
            resAfterOpt.print();
            std::cout << Color::def << std::endl;
        }
        PrintSchedule(tasksInfo, x);
    }

    /**
     * @brief Perform scheduling based on optimization
     *
     * @param tasks
     * @return VectorDynamic all the task instances' start time
     */
    OptimizeResult OptimizeScheduling(DAG_Model &dagTasks,
                                      size_t initialSeed = ResetInnerWeightLoopMax + 1,
                                      VectorDynamic initialUser = GenerateVectorDynamic(1))
    {
        TaskSet tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        // EliminationForest forestInfo(tasksInfo);

        VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);

        // initialEstimate << 0.000335766, 198.888, 202.048, 300, 454, 503, 52.0149, 304, 290, 301, 285, 302, 1, 203.997, 455;

        int loopNumber = 0;
        VectorDynamic bestResultFound;
        double prevError = INT32_MAX;
        GradientVanishPairs prevGVPair;

        // this makes sure we get the same result every time we run the program
        size_t prevSrandRef = initialSeed;
        ResetSRand(prevSrandRef);
        RelocationMethod currentRelocationMethod = EndOfLongInterval;

        StateActionCollection currAction;

        std::string pathFolder = PROJECT_PATH + "RL/";

        while (loopNumber < ResetInnerWeightLoopMax)
        {
            initialEstimate = AlignVariablesF2I(initialEstimate, RoundingThreshold);
            // perform optimization for one time
            if (debugMode == 1)
            {
                std::cout << "Initial solution for this loop: " << initialEstimate << std::endl;
            }
            Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
            NonlinearFactorGraph graph;
            BuildFactorGraph(dagTasks, graph, tasksInfo);
            Values result = SolveFactorGraph(graph, initialEstimateFG);
            VectorDynamic startTimeComplete = CollectUnitOptResult(result, tasksInfo);
            // TODO: probably add job aligns?
            PrintResultAnalyzation(dagTasks, tasksInfo, startTimeComplete);
            if (debugMode)
                std::cout << Color::green << "UnitOptimization finishes for one time" << Color::def << endl;

            // convergence check,
            double currError = GraphErrorEvaluation(dagTasks, startTimeComplete);
            currAction.currError = currError;
            // if (recordActionValue > 0)
            // {
            //     std::string path = pathFolder + "record" + std::to_string(recordRLFileCount) + ".txt";
            //     std::ifstream myfile;
            //     myfile.open(path);
            //     if (!myfile) // create the file
            //     {
            //         std::ofstream myfile;
            //         myfile.open(path, fstream::in | fstream::out | fstream::trunc);
            //         std::string info = ResizeStr("CurrError") + ResizeStr("VanishGrad") + ResizeStr("ResetWeight") + ResizeStr("ChangeInitial") + ResizeStr("RelocateInterval") + "\n";
            //         myfile << info;
            //         myfile.close();
            //     }
            //     std::ofstream myfileOutput;
            //     myfileOutput.open(path, std::ios_base::app);
            //     myfileOutput << currAction.to_string();
            //     myfileOutput.close();
            // }
            if (currError < 1e-3)
            {
                bestResultFound = startTimeComplete;
                break;
            }
            else if ((prevError - currError) / prevError > relativeErrorTolerance) // observable error decrease
            {
                ResetSRand(prevSrandRef); // use previous weights in the next iteration
                bestResultFound = startTimeComplete;
                prevError = currError;
            }
            else
            {
                CoutWarning("Error increased!");
                ResetSRand(++prevSrandRef); // try different random weights
            }
            currAction.resetWeightSeed = prevSrandRef;

            startTimeComplete = JobGroupsOptimize(startTimeComplete, tasksInfo, graph);

            // detect and handle gradient vanish
            GradientVanishDetectResult gvRes = RelocateIncludedInterval(tasksInfo, graph, startTimeComplete, currentRelocationMethod);
            if (prevGVPair == gvRes.gradientVanishPairs)
            {
                currentRelocationMethod = IncrementRelocationMethod(currentRelocationMethod);
                currAction.relocateMethod = currentRelocationMethod;
            }
            else
            {
                prevGVPair = gvRes.gradientVanishPairs;
            }
            initialEstimate = gvRes.startTimeVectorAfterRelocate;

            loopNumber++;
            if (loopNumber >= ResetInnerWeightLoopMax)
                CoutWarning("Loop number Warning in OptimizeScheduling");
        }

        initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
        if (PrintInitial)
        {
            std::cout << "The initial solution is: " << std::endl
                      << initialEstimate << std::endl;
        }
        double errorInitial = GraphErrorEvaluation(dagTasks, initialEstimate);

        cout << Color::blue << "The error before optimization is "
             << errorInitial << Color::def << endl;
        double finalError = GraphErrorEvaluation(dagTasks, bestResultFound);
        cout << Color::blue << "The error after optimization is " << finalError << Color::def << endl;

        return {errorInitial, finalError, initialEstimate, bestResultFound};
    }
    void inline donothing(const std::filesystem::directory_entry &p)
    {
        ;
    }
    OptimizeResult OptimizeSchedulingResetSeed(DAG_Model &dagTasks)
    {

        // if (recordActionValue > 0)
        // {
        //     std::filesystem::path p1{PROJECT_PATH + "RL/"};
        //     for (auto &p : std::filesystem::directory_iterator(p1))
        //     {
        //         recordRLFileCount++;
        //         donothing(p);
        //     }
        // }

        OptimizeResult sth;
        for (int i = 100; i < 100 + RandomDrawWeightMaxLoop; i++)
        {
            sth = OptimizeScheduling(dagTasks, i);

            VectorDynamic res = sth.optimizeVariable;
            // if (debugMode > 1)
            cout << "The result after optimization is " << Color::green << sth.optimizeError
                 << Color::def << endl;
            if (sth.optimizeError < 1e-3)
            {
                if (i > 101)
                {
                    std::cout << "Total re-seed times: " << i - 100 << std::endl;
                }
                break;
            }

            if (PrintOutput)
                cout << Color::blue << res << Color::def << endl;
        }
        return sth;
    }
}