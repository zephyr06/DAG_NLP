
#pragma once
#include "unordered_map"

#include "RegularTasks.h"
#include "DAG_Model.h"
#include "MakeSpanFactor.h"
#include "DAG_ConstraintFactor.h"
#include "DBF_ConstraintFactor.h"
#include "DDL_ConstraintFactor.h"
#include "SensorFusionFactor.h"
#include "PriorFactor.h"
#include "GraphUtilsFromBGL.h"
#include "colormod.h"

using namespace RegularTaskSystem;
// -------------------------------------------------------- from previous optimization begins

// -------------------------------------------------------- from previous optimization ends

namespace DAG_SPACE
{
    /**
     * @brief Generate initial solution for the whole optimization
     * 
     * @param tasks 
     * @param sizeOfVariables 
     * @return VectorDynamic size (N+1), first N is start time for nodes, the last one is r.h.s.
     */
    VectorDynamic GenerateInitialForDAG(DAG_Model &dagTasks, vector<LLint> &sizeOfVariables, int variableDimension)
    {
        int N = dagTasks.tasks.size();
        TaskSet &tasks = dagTasks.tasks;
        vector<int> order = FindDependencyOrder(dagTasks);
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);

        vector<double> executionTimeVec = GetParameter<double>(tasks, "executionTime");

        // m maps from tasks index to startTimeVector index
        std::unordered_map<int, LLint> m;
        LLint sumVariableNum = 0;
        for (int i = 0; i < N; i++)
        {
            m[i] = sumVariableNum;
            sumVariableNum += sizeOfVariables[i];
        }

        LLint index = 0;
        for (int i = 0; i < N; i++)
        {
            int currTaskIndex = order[i];
            for (int j = 0; j < sizeOfVariables[currTaskIndex]; j++)
            {
                initial(m[currTaskIndex] + j, 0) = j * tasks[currTaskIndex].period + index++;
            }
        }
        // initial(N, 0) *= 1;
        // initial << 1, 101, 202, 303, 4, 5, 206, 50, 258, 67;
        // initial << 0, 9, 17, 19, 25;
        // initial << 1, 5, 2, 3, 0;
        // initial << 4, 3, 2, 1, 0;
        // initial << 6, 2.5, 2, 0.5, 0;
        // initial << 1, 100, 0, 0, 0, 0;
        // cout << "Initial estimate is " << initial << endl;
        return initial;
    }

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

    /**
     * @brief Create a Scheduling Graph object;
     * somehow, the returned graph doesn't contain passed parameters
     * 
     * @param dagTasks 
     * @param initialEstimate 
     * @param mapIndex 
     * @param maskForEliminate 
     * @param sizeOfVariables 
     * @param variableDimension 
     * @param hyperPeriod 
     * @return NonlinearFactorGraph 
     */
    NonlinearFactorGraph CreateSchedulingGraph(DAG_Model &dagTasks, VectorDynamic &initialEstimate,
                                               MAP_Index2Data mapIndex, vector<bool> &maskForEliminate,
                                               vector<LLint> &sizeOfVariables, int variableDimension,
                                               LLint hyperPeriod)
    {
        using namespace RegularTaskSystem;

        int N = dagTasks.tasks.size();

        // build the factor graph
        NonlinearFactorGraph graph;
        Symbol key('a', 0);

        LLint errorDimensionMS = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
        graph.emplace_shared<MakeSpanFactor>(key, dagTasks, sizeOfVariables,
                                             errorDimensionMS, mapIndex,
                                             maskForEliminate, model);

        LLint errorDimensionDAG = dagTasks.edgeNumber();
        model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                   errorDimensionDAG, mapIndex,
                                                   maskForEliminate, model);

        LLint errorDimensionDBF = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDBF, mapIndex,
                                                   maskForEliminate,
                                                   model);

        LLint errorDimensionDDL = 2 * variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDDL, mapIndex,
                                                   maskForEliminate, model);

        LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                            errorDimensionSF, sensorFusionTolerance,
                                                            mapIndex, maskForEliminate, model);
        return graph;
    }

    double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector)
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

        MAP_Index2Data mapIndex;
        for (LLint i = 0; i < variableDimension; i++)
        {
            MappingDataStruct m{i, 0};
            mapIndex[i] = m;
        }
        bool whetherEliminate = false;
        vector<bool> maskForEliminate(variableDimension, false);
        // build the factor graph
        // NonlinearFactorGraph graph = CreateSchedulingGraph(dagTasks, startTimeVector,
        //                                                    mapIndex, maskForEliminate,
        //                                                    sizeOfVariables, variableDimension,
        //                                                    hyperPeriod);

        Symbol key('a', 0);
        NonlinearFactorGraph graph;
        LLint errorDimensionMS = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
        graph.emplace_shared<MakeSpanFactor>(key, dagTasks, sizeOfVariables,
                                             errorDimensionMS, mapIndex,
                                             maskForEliminate, model);

        LLint errorDimensionDAG = dagTasks.edgeNumber();
        model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                   errorDimensionDAG, mapIndex,
                                                   maskForEliminate, model);

        LLint errorDimensionDBF = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDBF, mapIndex,
                                                   maskForEliminate,
                                                   model);

        LLint errorDimensionDDL = 2 * variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDDL, mapIndex,
                                                   maskForEliminate, model);
        LLint errorDimensionPrior = 1;
        vector<int> order = FindDependencyOrder(dagTasks);
        model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
        graph.emplace_shared<Prior_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                     errorDimensionPrior, mapIndex,
                                                     maskForEliminate, 0.0, order[0], model);

        LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                            errorDimensionSF, sensorFusionTolerance,
                                                            mapIndex, maskForEliminate, model);
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
    pair<VectorDynamic, NonlinearFactorGraph> UnitOptimization(DAG_Model &dagTasks, VectorDynamic &initialEstimate,
                                                               MAP_Index2Data mapIndex, vector<bool> &maskForEliminate,
                                                               vector<LLint> &sizeOfVariables, int variableDimension,
                                                               LLint hyperPeriod)
    {
        using namespace RegularTaskSystem;

        int N = dagTasks.tasks.size();

        // build the factor graph
        // Symbol key('a', 0);
        // NonlinearFactorGraph graph = CreateSchedulingGraph(dagTasks, initialEstimate,
        //                                                    mapIndex, maskForEliminate,
        //                                                    sizeOfVariables, variableDimension,
        //                                                    hyperPeriod);

        // build the factor graph
        NonlinearFactorGraph graph;
        Symbol key('a', 0);

        LLint errorDimensionMS = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
        graph.emplace_shared<MakeSpanFactor>(key, dagTasks, sizeOfVariables,
                                             errorDimensionMS, mapIndex,
                                             maskForEliminate, model);

        LLint errorDimensionDAG = dagTasks.edgeNumber();
        model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                   errorDimensionDAG, mapIndex,
                                                   maskForEliminate, model);

        LLint errorDimensionDBF = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDBF, mapIndex,
                                                   maskForEliminate,
                                                   model);

        LLint errorDimensionDDL = 2 * variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                   errorDimensionDDL, mapIndex,
                                                   maskForEliminate, model);

        LLint errorDimensionPrior = 1;
        vector<int> order = FindDependencyOrder(dagTasks);
        model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
        graph.emplace_shared<Prior_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                     errorDimensionPrior, mapIndex,
                                                     maskForEliminate, 0.0, order[0], model);

        LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
        model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                            errorDimensionSF, sensorFusionTolerance,
                                                            mapIndex, maskForEliminate, model);
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
                params.setVerbosityLM("TRYDELTA");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            params.setMaxIterations(maxIterations);
            params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
        cout << Color::green << "UnitOptimization finishes for one time" << Color::def << endl;
        return make_pair(optComp, graph);
    }

    /**
     * @brief Perform scheduling based on optimization
     * 
     * @param tasks 
     * @return VectorDynamic all the task instances' start time
     */
    pair<double, VectorDynamic> OptimizeScheduling(DAG_Model &dagTasks)
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
        VectorDynamic initialEstimate = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);

        MAP_Index2Data mapIndex;
        for (LLint i = 0; i < variableDimension; i++)
        {
            MappingDataStruct m{i, 0};
            mapIndex[i] = m;
        };

        bool whetherEliminate = false;
        int loopNumber = 0;
        vector<bool> maskForEliminate(variableDimension, false);
        VectorDynamic resTemp = GenerateVectorDynamic(variableDimension);
        // build elimination eliminationTrees
        pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(dagTasks);
        Graph eliminationTrees = sth.first;
        indexVertexMap indexesBGL = sth.second;
        VectorDynamic trueResult;
        while (1)
        {
            whetherEliminate = false;
            // TODO: modify interface later
            auto sth = UnitOptimization(dagTasks, initialEstimate,
                                        mapIndex, maskForEliminate,
                                        sizeOfVariables, variableDimension,
                                        hyperPeriod);

            resTemp = sth.first;
            // graph = sth.second;
            VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, maskForEliminate, mapIndex);

            // factors that require elimination analysis are: DBF
            LLint errorDimensionDBF = 1;
            auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            Symbol key('b', 0);
            DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                        mapIndex, maskForEliminate, model);
            // this function performs in-place modification for all the variables!
            // TODO: should we add eliminate function for sensorFusion?
            factor.addMappingFunction(resTemp, mapIndex, whetherEliminate, maskForEliminate,
                                      eliminationTrees, indexesBGL);
            // update initial estimate
            vector<double> initialUpdateVec;
            initialUpdateVec.reserve(variableDimension - 1);
            LLint indexUpdate = 0;
            for (size_t i = 0; i < variableDimension; i++)
            {
                if (not maskForEliminate[i])
                {
                    initialUpdateVec.push_back(startTimeComplete(i, 0));
                }
            }
            VectorDynamic initialUpdate;
            initialUpdate.resize(initialUpdateVec.size(), 1);
            for (size_t i = 0; i < initialUpdateVec.size(); i++)
            {
                initialUpdate(i, 0) = initialUpdateVec[i];
            }
            initialEstimate = initialUpdate;

            if (not whetherEliminate)
            {
                trueResult = RecoverStartTimeVector(resTemp, maskForEliminate, mapIndex);
                break;
            }
            loopNumber++;
            if (loopNumber > N)
            {
                CoutWarning("Loop number Warning in OptimizeScheduling");
                break;
            }
        }

        initialEstimate = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
        cout << Color::blue << "The error before optimization is "
             << GraphErrorEvaluation(dagTasks, initialEstimate) << Color::def << endl;
        double finalError = GraphErrorEvaluation(dagTasks, trueResult);
        cout << Color::blue << "The error after optimization is " << finalError << Color::def << endl;
        return make_pair(finalError, trueResult);
    }
}