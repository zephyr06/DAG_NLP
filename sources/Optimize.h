#include <boost/function.hpp>
#include "RegularTasks.h"
#include "DAG_ConstraintFactor.h"
#include "DBF_ConstraintFactor.h"
#include "DDL_ConstraintFactor.h"
#include "SensorFusionFactor.h"

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
    VectorDynamic GenerateInitialForDAG(TaskSet &tasks, vector<LLint> &sizeOfVariables, int variableDimension)
    {
        int N = tasks.size();
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);
        vector<double> executionTimeVec = GetParameter<double>(tasks, "executionTime");
        LLint index = 0;
        // for (int i = 0; i < N; i++)
        // {
        //     for (int j = 0; j < sizeOfVariables[i]; j++)
        //     {
        //         initial(index++, 0) = j * tasks[i].period + index;
        //     }
        // }
        // initial(N, 0) *= 1;
        initial << 1, 101, 202, 303, 4, 5, 206, 50, 258, 67;
        // initial << 0, 0, 0, 0, 0, 0;
        // initial << 6, 2.5, 2, 0.5, 0;
        // initial << 1, 100, 0, 0, 0, 0;
        return initial;
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
    pair<VectorDynamic, NonlinearFactorGraph> UnitOptimization(TaskSet &tasks, VectorDynamic &initialEstimate,
                                                               MAP_Index2Data mapIndex, vector<bool> &maskForEliminate,
                                                               vector<LLint> &sizeOfVariables, int variableDimension,
                                                               LLint hyperPeriod)
    {
        using namespace RegularTaskSystem;

        int N = tasks.size();

        // build the factor graph
        NonlinearFactorGraph graph;
        Symbol key('a', 0);

        LLint errorDimensionDAG = 1 + 4;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, tasks, sizeOfVariables,
                                                   errorDimensionDAG, mapIndex,
                                                   maskForEliminate, model);
        LLint errorDimensionDBF = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, tasks, sizeOfVariables,
                                                   errorDimensionDBF, mapIndex,
                                                   maskForEliminate,
                                                   model);
        LLint errorDimensionDDL = 2 * variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, tasks, sizeOfVariables,
                                                   errorDimensionDDL, mapIndex,
                                                   maskForEliminate, model);
        LLint errorDimensionSF = sizeOfVariables[3];
        model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        graph.emplace_shared<SensorFusion_ConstraintFactor>(key, tasks, sizeOfVariables,
                                                            errorDimensionSF, sensorFusionTolerance,
                                                            mapIndex, maskForEliminate, model);

        Values initialEstimateFG;
        initialEstimateFG.insert(key, initialEstimate);

        Values result;
        if (optimizerType == 1)
        {
            DoglegParams params;
            if (debugMode == 1)
                params.setVerbosityDL("VERBOSE");
            params.setDeltaInitial(deltaInitialDogleg);
            params.setRelativeErrorTol(relativeErrorTolerance);
            DoglegOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 2)
        {
            LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            if (debugMode == 1)
                params.setVerbosityLM("SUMMARY");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
        cout << green << "UnitOptimization finishes for one time" << def << endl;
        return make_pair(optComp, graph);
    }

    /**
     * @brief Perform scheduling based on optimization
     * 
     * @param tasks 
     * @return VectorDynamic all the task instances' start time
     */
    VectorDynamic OptimizeScheduling(TaskSet &tasks)
    {
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
        VectorDynamic initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables, variableDimension);

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
        NonlinearFactorGraph graph;
        VectorDynamic trueResult;
        while (1)
        {
            whetherEliminate = false;
            // TODO: modify interface later
            auto sth = UnitOptimization(tasks, initialEstimate,
                                        mapIndex, maskForEliminate,
                                        sizeOfVariables, variableDimension,
                                        hyperPeriod);

            resTemp = sth.first;
            graph = sth.second;
            VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, maskForEliminate, mapIndex);

            // factors that require elimination analysis are: DBF
            LLint errorDimensionDBF = 1;
            auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            Symbol key('b', 0);
            DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                        mapIndex, maskForEliminate, model);
            // this function performs in-place modification for all the variables!
            factor.addMappingFunction(resTemp, mapIndex, whetherEliminate, maskForEliminate);
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
                cout << "Loop number error in OptimizeScheduling" << endl;
                throw;
            }
        }
        Symbol key('a', 0);

        Values finalEstimate;
        finalEstimate.insert(key, trueResult);
        Values initialEstimateFG;
        initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables, variableDimension);
        initialEstimateFG.insert(key, initialEstimate);
        // cout << blue << "The error before optimization is " << graph.error(initialEstimateFG) << def << endl;
        // cout << blue << "The error after optimization is " << graph.error(finalEstimate) << def << endl;

        return trueResult;
    }
}