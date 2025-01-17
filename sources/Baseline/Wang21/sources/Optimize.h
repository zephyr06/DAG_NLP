
#pragma once
#include "unordered_map"

#include "Parameters.h"

#include "sources/TaskModel/RegularTasks.h"

#include "sources/TaskModel/DAG_Model.h"
#include "MakeSpanFactor.h"
#include "DAG_ConstraintFactor.h"
#include "DBF_ConstraintFactor.h"
#include "DDL_ConstraintFactor.h"
#include "SensorFusionFactor.h"
#include "PriorFactor.h"
#include "GraphUtilsFromBGL.h"
#include "InitialEstimate.h"
#include "colormod.h"

namespace RTSS21IC_NLP
{

    using namespace RegularTaskSystem;
    // -------------------------------------------------------- from previous optimization begins

    // -------------------------------------------------------- from previous optimization ends

    namespace DAG_SPACE
    {

        LLint CountSFError(OrderOptDAG_SPACE::DAG_Model &dagTasks, std::vector<LLint> &sizeOfVariables)
        {
            LLint errorDimensionSF = 0;
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                if ((itr->second).size() > 1)
                    errorDimensionSF += sizeOfVariables[(itr->first)];
            }
            return errorDimensionSF;
        }

        double GraphErrorEvaluation(OrderOptDAG_SPACE::DAG_Model &dagTasks, VectorDynamic startTimeVector)
        {
            TaskSet tasks = dagTasks.tasks;
            int N = tasks.size();
            LLint hyperPeriod = HyperPeriod(tasks);

            // declare variables
            std::vector<LLint> sizeOfVariables;
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
            std::vector<bool> maskForEliminate(variableDimension, false);

            gtsam::Symbol key('a', 0);
            NonlinearFactorGraph graph;
            LLint errorDimensionMS = 1;

            LLint errorDimensionDAG = dagTasks.edgeNumber();
            auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
            graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                       errorDimensionDAG, mapIndex,
                                                       maskForEliminate, model);
            if (makespanWeight > 0)
            {
                LLint errorDimensionMS = 1;
                model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
                graph.emplace_shared<MakeSpanFactor>(key, dagTasks, sizeOfVariables,
                                                     errorDimensionMS, mapIndex,
                                                     maskForEliminate, model);
            }

            ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
            LLint errorDimensionDBF = processorTaskSet.size();
            model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                       errorDimensionDBF, mapIndex,
                                                       maskForEliminate, processorTaskSet,
                                                       model);

            LLint errorDimensionDDL = 2 * variableDimension;
            model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
            graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                       errorDimensionDDL, mapIndex,
                                                       maskForEliminate, model);

            if (weightPrior_factor > 0)
            {
                LLint errorDimensionPrior = 1;
                std::vector<int> order = FindDependencyOrder(dagTasks);
                model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
                graph.emplace_shared<Prior_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                             errorDimensionPrior, mapIndex,
                                                             maskForEliminate, 0.0, order[0], model);
            }

            LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
            model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
            graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                                errorDimensionSF, sensorFusionTolerance,
                                                                mapIndex, maskForEliminate, model);
            gtsam::Values initialEstimateFG;
            // gtsam::Symbol key('a', 0);
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
        VectorDynamic UnitOptimization(OrderOptDAG_SPACE::DAG_Model &dagTasks, VectorDynamic &initialEstimate,
                                       MAP_Index2Data mapIndex, std::vector<bool> &maskForEliminate,
                                       std::vector<LLint> &sizeOfVariables, int variableDimension,
                                       LLint hyperPeriod)
        {
            using namespace RegularTaskSystem;

            int N = dagTasks.tasks.size();

            // build the factor graph
            NonlinearFactorGraph graph;
            gtsam::Symbol key('a', 0);

            LLint errorDimensionDAG = dagTasks.edgeNumber();
            auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
            graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                       errorDimensionDAG, mapIndex,
                                                       maskForEliminate, model);
            if (makespanWeight > 0)
            {
                LLint errorDimensionMS = 1;
                model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
                graph.emplace_shared<MakeSpanFactor>(key, dagTasks, sizeOfVariables,
                                                     errorDimensionMS, mapIndex,
                                                     maskForEliminate, model);
            }

            ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
            LLint errorDimensionDBF = processorTaskSet.size();
            model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
            graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                       errorDimensionDBF, mapIndex,
                                                       maskForEliminate, processorTaskSet,
                                                       model);

            LLint errorDimensionDDL = 2 * variableDimension;
            model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
            graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                       errorDimensionDDL, mapIndex,
                                                       maskForEliminate, model);

            if (weightPrior_factor > 0)
            {
                LLint errorDimensionPrior = 1;
                std::vector<int> order = FindDependencyOrder(dagTasks);
                model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
                graph.emplace_shared<Prior_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                             errorDimensionPrior, mapIndex,
                                                             maskForEliminate, 0.0, order[0], model);
            }

            LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
            model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
            graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                                errorDimensionSF, sensorFusionTolerance,
                                                                mapIndex, maskForEliminate, model);
            // return graph;

            gtsam::Values initialEstimateFG;
            initialEstimateFG.insert(key, initialEstimate);

            gtsam::Values result;
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
                std::cout << Color::green << "UnitOptimization finishes for one time" << Color::def << std::endl;
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
        OptimizeResult InitialScheduling(OrderOptDAG_SPACE::DAG_Model &dagTasks)
        {
            TaskSet tasks = dagTasks.tasks;
            int N = tasks.size();
            LLint hyperPeriod = HyperPeriod(tasks);

            // declare variables
            std::vector<LLint> sizeOfVariables;
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

        VectorDynamic RandomWalk(VectorDynamic &startTimeVector, OrderOptDAG_SPACE::DAG_Model &dagTasks,
                                 Graph &eliminationTrees, indexVertexMap &indexesBGL)
        {
            // IntervalTree={}
            // for each interval intV:
            //     if interval
            return startTimeVector;
        }
        /**
         * @brief Perform scheduling based on optimization
         *
         * @param tasks
         * @return VectorDynamic all the task instances' start time
         */
        OptimizeResult OptimizeScheduling(OrderOptDAG_SPACE::DAG_Model &dagTasks, VectorDynamic initialUser = GenerateVectorDynamic(1))
        {
            TaskSet tasks = dagTasks.tasks;
            int N = tasks.size();
            LLint hyperPeriod = HyperPeriod(tasks);

            // declare variables
            std::vector<LLint> sizeOfVariables;
            int variableDimension = 0;
            for (int i = 0; i < N; i++)
            {
                LLint size = hyperPeriod / tasks[i].period;
                sizeOfVariables.push_back(size);
                variableDimension += size;
            }

            VectorDynamic initialEstimate = GenerateInitial(dagTasks, sizeOfVariables, variableDimension, initialUser);

            MAP_Index2Data mapIndex;
            for (LLint i = 0; i < variableDimension; i++)
            {
                MappingDataStruct m{i, 0};
                mapIndex[i] = m;
            };

            bool whetherEliminate = false;
            int loopNumber = 0;
            std::vector<bool> maskForEliminate(variableDimension, false);
            VectorDynamic resTemp = GenerateVectorDynamic(variableDimension);
            // build elimination eliminationTrees
            std::pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(dagTasks);
            Graph eliminationTrees = sth.first;
            indexVertexMap indexesBGL = sth.second;
            VectorDynamic trueResult;
            while (1)
            {
                whetherEliminate = false;
                BeginTimer("UnitOptimization");
                resTemp = UnitOptimization(dagTasks, initialEstimate,
                                           mapIndex, maskForEliminate,
                                           sizeOfVariables, variableDimension,
                                           hyperPeriod);
                EndTimer("UnitOptimization");
                VectorDynamic startTimeComplete = RecoverStartTimeVector(resTemp, maskForEliminate, mapIndex);

                startTimeComplete = RandomWalk(startTimeComplete, dagTasks, eliminationTrees, indexesBGL);
                // factors that require elimination analysis are: DBF
                ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
                LLint errorDimensionDBF = processorTaskSet.size();

                auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
                gtsam::Symbol key('b', 0);
                DBF_ConstraintFactor factor(key, tasks, sizeOfVariables, errorDimensionDBF,
                                            mapIndex, maskForEliminate, processorTaskSet, model);
                // this function performs in-place modification for all the variables!
                // TODO(Old): should we add eliminate function for sensorFusion?
                factor.addMappingFunction(resTemp, mapIndex, whetherEliminate, maskForEliminate,
                                          eliminationTrees, indexesBGL);
                // update initial estimate
                std::vector<double> initialUpdateVec;
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
                if (loopNumber > ElimnateLoop_Max)
                {
                    CoutWarning("Loop number Warning in OptimizeScheduling");
                    // cannot use mapIndex to recover, because mapIndex has already been changed at this point
                    trueResult = startTimeComplete;
                    break;
                }
            }

            initialEstimate = GenerateInitial(dagTasks, sizeOfVariables, variableDimension, initialUser);
            double errorInitial = GraphErrorEvaluation(dagTasks, initialEstimate);

            std::cout << Color::blue << "The error before optimization is "
                      << errorInitial << Color::def << std::endl;
            double finalError = GraphErrorEvaluation(dagTasks, trueResult);
            std::cout << Color::blue << "The error after optimization is " << finalError << Color::def << std::endl;
            return {errorInitial, finalError, initialEstimate, trueResult};
        }
    }
} // namespace RTSS21IC_NLP
