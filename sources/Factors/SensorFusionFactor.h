/**
 * @file SensorFusionFactor.h
 * @author Sen Wang
 * @brief This file implements sensor fusion factor for non-preemptive case;
 * @version 0.1
 * @date 2022-06-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "sources/Utils/DeclareDAG.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/BaseSchedulingFactor.h"
#include "sources/Optimization/JobOrder.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    LLint CountSFError(const DAG_Model &dagTasks, const vector<LLint> &sizeOfVariables)
    {
        LLint errorDimensionSF = 0;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            if ((itr->second).size() > 1)
                errorDimensionSF += sizeOfVariables[(itr->first)];
        }
        return errorDimensionSF;
    }

    struct IndexData
    {
        LLint index;
        double time;
    };
    pair<int, int> ExtractMaxDistance(vector<IndexData> &sourceFinishTime)
    {
        // return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
        //        *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
        if (sourceFinishTime.size() == 0)
            return make_pair(0, 0);

        double maxEle = sourceFinishTime[0].time;
        int maxIndex = 0;
        double minEle = sourceFinishTime[0].time;
        int minIndex = 0;

        for (uint i = 0; i < sourceFinishTime.size(); i++)
        {
            if (maxEle < sourceFinishTime[i].time)
            {
                maxIndex = i;
                maxEle = sourceFinishTime[i].time;
            }
            if (minEle > sourceFinishTime[i].time)
            {
                minIndex = i;
                minIndex = sourceFinishTime[i].time;
            }
        }
        return make_pair(maxIndex, minIndex);
    }
    double ExtractMaxDistance(vector<double> &sourceFinishTime)
    {
        return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
               *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
    }

    VectorDynamic ObtainSensorFusionError(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector)
    {
        VectorDynamic res;
        LLint errorDimension = CountSFError(dagTasks, tasksInfo.sizeOfVariables);
        if (errorDimension == 0)
            return GenerateVectorDynamic1D(0);

        res.resize(errorDimension, 1);
        LLint indexRes = 0;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            if (tasksPrev.size() > 1)
            {
                vector<double> sourceFinishTime;
                sourceFinishTime.reserve(tasksPrev.size());
                size_t indexCurr = itr->first;

                for (int instanceCurr = 0; instanceCurr < tasksInfo.sizeOfVariables[indexCurr]; instanceCurr++)
                {
                    sourceFinishTime.clear();
                    double startTimeCurr = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, indexCurr, instanceCurr);

                    // go through three source sensor tasks in the example
                    for (size_t ii = 0; ii < tasksPrev.size(); ii++)
                    {
                        int sourceIndex = tasksPrev.at(ii).id;
                        LLint instanceSource = floor(startTimeCurr / tasksInfo.tasks[sourceIndex].period);
                        if (instanceSource < 0 || instanceSource > tasksInfo.sizeOfVariables[sourceIndex] - 1)
                            CoutError("Error in OptimizeORder's SF evaluation!");

                        JobCEC jCurr(sourceIndex, instanceSource);
                        double finishTimeSourceInstance = GetFinishTime(jCurr, startTimeVector, tasksInfo);
                        if (finishTimeSourceInstance <= startTimeCurr)
                            sourceFinishTime.push_back(finishTimeSourceInstance);
                        else
                        {
                            if (jCurr.jobId == 0)
                                jCurr.jobId = tasksInfo.sizeOfVariables[jCurr.taskId] - 1;
                            else
                                jCurr.jobId--;
                            sourceFinishTime.push_back(GetFinishTime(jCurr, startTimeVector, tasksInfo));
                        }
                    }
                    res(indexRes, 0) = ExtractMaxDistance(sourceFinishTime);
                    indexRes++;
                }
            }
        }
        return res;
    }

    std::vector<gtsam::Symbol> GenerateKeySF(TaskSetInfoDerived &tasksInfo)
    {
        std::vector<gtsam::Symbol> keyVec;
        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (LLint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                keyVec.push_back(GenerateKey(i, j));
            }
        }
        return keyVec;
    }
    void AddSF_Factor(gtsam::NonlinearFactorGraph &graph, DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo)
    {

        if (weightSF_factor == 0)
            return;
        LLint errorDimensionSF = CountSFError(dagTasks, tasksInfo.sizeOfVariables);
        if (errorDimensionSF == 0)
            return;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma / weightSF_factor);
        vector<gtsam::Symbol> keys = GenerateKeySF(tasksInfo);

        LambdaMultiKey f = [dagTasks, tasksInfo](const Values &x)
        {
            VectorDynamic startTimeVector = GenerateVectorDynamic(tasksInfo.length);
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (LLint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                    startTimeVector(IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables)) = x.at<VectorDynamic>(GenerateKey(i, j))(0);
            }
            VectorDynamic res = ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);
            for (uint i = 0; i < res.rows(); i++)
                res(i) = Barrier(sensorFusionTolerance - res(i));
            return res;
        };
        graph.emplace_shared<MultiKeyFactor>(keys, f, errorDimensionSF, model);
    }

    class SensorFusion_ConstraintFactor : public BaseSchedulingFactor
    {
    public:
        DAG_Model dagTasks;
        double sensorFusionTol; // not used, actually

        SensorFusion_ConstraintFactor(Key key, DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo,
                                      EliminationForest &forestInfo,
                                      LLint errorDimension,
                                      double sensorFusionTol,
                                      SharedNoiseModel model) : BaseSchedulingFactor(key, tasksInfo, forestInfo, errorDimension, model),
                                                                dagTasks(dagTasks),
                                                                sensorFusionTol(sensorFusionTol)

        {
        }
        // TODO: design some tests for SensorFusion
        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {
            BeginTimer("Sensor_all");

            if (H)
            {
                if (numericalJaobian)
                {
                    *H = NumericalDerivativeDynamic(f, startTimeVector, deltaOptimizer, errorDimension);
                }
                else
                    *H = JacobianAnalytic(startTimeVector);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of SensorFusion_ConstraintFactor is " << endl;
                    cout << *H << endl;
                }
                if (debugMode == 1)
                {
                    cout << "The error vector of SensorFusion is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }
            EndTimer("Sensor_all");
            return f(startTimeVector);
        }

        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            // go through all the instances of task, 3 in the example DAG
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                const TaskSet &tasksPrev = itr->second;
                if (tasksPrev.size() > 1)
                {
                    vector<double> sourceFinishTime;
                    sourceFinishTime.reserve(tasksPrev.size());
                    size_t indexCurr = itr->first;

                    for (int instanceCurr = 0; instanceCurr < tasksInfo.sizeOfVariables[indexCurr]; instanceCurr++)
                    {
                        boost::function<double()> funcLocal =
                            [&]()
                        {
                            sourceFinishTime.clear();

                            // if DAG constraints are violated, addedErrorDAG will count the violated part
                            double addedErrorDAG = 0;
                            double addedErrorDDL = 0;
                            double startTimeCurr = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, indexCurr, instanceCurr);

                            // go through three source sensor tasks in the example
                            for (size_t ii = 0; ii < tasksPrev.size(); ii++)
                            {
                                int sourceIndex = tasksPrev.at(ii).id;
                                LLint instanceSource = floor(startTimeCurr / tasksInfo.tasks[sourceIndex].period);
                                // if instanceSource<0, that means startTimeCurr <0, we'll use the first sourceInstance in a period instead
                                // instanceSource = (instanceSource < 0) ? 0 : instanceSource;
                                if (instanceSource < 0)
                                    instanceSource = 0;
                                else if (instanceSource > tasksInfo.sizeOfVariables[sourceIndex] - 1)
                                    instanceSource = tasksInfo.sizeOfVariables[sourceIndex] - 1;

                                double startTimeSourceInstance = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables,
                                                                                 sourceIndex, instanceSource);
                                double finishTimeSourceInstance = startTimeSourceInstance + tasksInfo.tasks[sourceIndex].executionTime;
                                if (finishTimeSourceInstance <= startTimeCurr)
                                    sourceFinishTime.push_back(finishTimeSourceInstance);
                                else if (instanceSource - 1 >= 0)
                                {
                                    double startTime_k_l_prev = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables,
                                                                                sourceIndex, instanceSource - 1);
                                    if (startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime >= startTimeCurr)
                                    {
                                        // in this case, self deadline constraint is violated, and so we'll just add addedErrorDDL
                                        addedErrorDDL += startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime - startTimeCurr;
                                    }
                                    sourceFinishTime.push_back(startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime);
                                }
                                // there is no previous instance, i.e., DAG constraints are violated,
                                // instanceSource = 0
                                // find a later instance, and return it because it will give a bigger error
                                else
                                {
                                    double startTime_k_l_next = startTimeSourceInstance;

                                    sourceFinishTime.push_back(startTime_k_l_next + tasksInfo.tasks[sourceIndex].executionTime);
                                    // if all the sources violate DAG constraints, this error will drag them back
                                    if (withAddedSensorFusionError)
                                    {
                                        // if (addedErrorDAG == 0)
                                        addedErrorDAG += startTime_k_l_next + tasksInfo.tasks[sourceIndex].executionTime - startTimeCurr;
                                        // else
                                        //     addedErrorDAG = min(addedErrorDAG, startTime_k_l_next +
                                        //                                            tasks[sourceIndex].executionTime - startTimeCurr);
                                    }
                                }
                            }
                            // this error is not well tested yet
                            double sensorFreshError = Barrier(FreshTol - (startTimeCurr -
                                                                          *min_element(sourceFinishTime.begin(),
                                                                                       sourceFinishTime.end())));
                            // double sensorFreshError = 0;

                            return addedErrorDDL + addedErrorDAG +
                                   Barrier(sensorFusionTolerance - ExtractMaxDistance(sourceFinishTime)) +
                                   sensorFreshError;
                        };

                        res(indexRes, 0) = funcLocal();
                        indexRes++;
                    }
                }
            }

            return res;
        };

        MatrixDynamic
        JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            BeginTimer("SF_H");
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);

            // int m = errorDimension;
            // y -> x
            SM_Dynamic j_yx(errorDimension, tasksInfo.length);
            LLint indexRes = 0;
            // go through all the tasks that needs sensor fusion, 3 in the example DAG
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                const TaskSet &tasksPrev = itr->second;
                if (tasksPrev.size() > 1)
                {
                    vector<double> sourceFinishTime;
                    sourceFinishTime.reserve(tasksPrev.size());
                    size_t indexCurr = itr->first;
                    for (int instanceCurr = 0; instanceCurr < tasksInfo.sizeOfVariables[indexCurr]; instanceCurr++)
                    {

                        boost::function<double()> funcLocal =
                            [&]()
                        {
                            sourceFinishTime.clear();

                            // if DAG constraints are violated, addedErrorDAG will count the violated part
                            double addedErrorDAG = 0;
                            double addedErrorDDL = 0;
                            double startTimeCurr = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, indexCurr, instanceCurr);

                            // go through three source sensor tasks in the example
                            for (size_t ii = 0; ii < tasksPrev.size(); ii++)
                            {
                                int sourceIndex = tasksPrev.at(ii).id;
                                LLint instanceSource = floor(startTimeCurr / tasksInfo.tasks[sourceIndex].period);
                                // if instanceSource<0, that means startTimeCurr <0, we'll use the first sourceInstance in a period instead
                                // instanceSource = (instanceSource < 0) ? 0 : instanceSource;
                                if (instanceSource < 0)
                                    instanceSource = 0;
                                else if (instanceSource > tasksInfo.sizeOfVariables[sourceIndex] - 1)
                                    instanceSource = tasksInfo.sizeOfVariables[sourceIndex] - 1;

                                double startTimeSourceInstance = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables,
                                                                                 sourceIndex, instanceSource);
                                double finishTimeSourceInstance = startTimeSourceInstance + tasksInfo.tasks[sourceIndex].executionTime;
                                if (finishTimeSourceInstance <= startTimeCurr)
                                    sourceFinishTime.push_back(finishTimeSourceInstance);
                                else if (instanceSource - 1 >= 0)
                                {
                                    double startTime_k_l_prev = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables,
                                                                                sourceIndex, instanceSource - 1);
                                    if (startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime >= startTimeCurr)
                                    {
                                        // in this case, self deadline constraint is violated, and so we'll just add addedErrorDDL
                                        addedErrorDDL += startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime - startTimeCurr;
                                    }
                                    sourceFinishTime.push_back(startTime_k_l_prev + tasksInfo.tasks[sourceIndex].executionTime);
                                }
                                // there is no previous instance, i.e., DAG constraints are violated,
                                // instanceSource = 0
                                // find a later instance, and return it because it will give a bigger error
                                else
                                {
                                    double startTime_k_l_next = startTimeSourceInstance;

                                    sourceFinishTime.push_back(startTime_k_l_next + tasksInfo.tasks[sourceIndex].executionTime);
                                    // if all the sources violate DAG constraints, this error will drag them back
                                    if (withAddedSensorFusionError)
                                    {
                                        // if (addedErrorDAG == 0)
                                        addedErrorDAG += startTime_k_l_next + tasksInfo.tasks[sourceIndex].executionTime - startTimeCurr;
                                        // else
                                        //     addedErrorDAG = min(addedErrorDAG, startTime_k_l_next +
                                        //                                            tasks[sourceIndex].executionTime - startTimeCurr);
                                    }
                                }
                            }
                            // this error is not well tested yet
                            double sensorFreshError = Barrier(FreshTol - (startTimeCurr +
                                                                          tasksInfo.tasks[indexCurr].executionTime -
                                                                          *min_element(sourceFinishTime.begin(),
                                                                                       sourceFinishTime.end())));

                            // double sensorFreshError = 0;
                            return addedErrorDDL + addedErrorDAG +
                                   Barrier(sensorFusionTolerance - ExtractMaxDistance(sourceFinishTime)) +
                                   sensorFreshError;
                        };

                        boost::function<void(LLint index_source_overall)> jacobianLocal =
                            [&](LLint index_source_overall)
                        {
                            if (index_source_overall < 0 || index_source_overall > startTimeVector.size())
                                return;
                            startTimeVector(index_source_overall, 0) += deltaOptimizer;
                            double errorPlus = funcLocal();
                            startTimeVector(index_source_overall, 0) -= 2 * deltaOptimizer;
                            double errorMinus = funcLocal();
                            UpdateSM((errorPlus - errorMinus) / 2 / deltaOptimizer, indexRes, index_source_overall, j_yx);
                            startTimeVector(index_source_overall, 0) += deltaOptimizer;
                        };

                        double startTimeCurr = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, indexCurr, instanceCurr);
                        for (size_t ii = 0; ii < tasksPrev.size(); ii++)
                        {
                            int sourceIndex = tasksPrev.at(ii).id;
                            LLint instanceSource = floor(startTimeCurr / tasksInfo.tasks[sourceIndex].period);
                            if (instanceSource < 0)
                                instanceSource = 0;
                            else if (instanceSource > tasksInfo.sizeOfVariables[sourceIndex] - 1)
                                instanceSource = tasksInfo.sizeOfVariables[sourceIndex] - 1;
                            LLint index_source_overall = IndexTran_Instance2Overall(sourceIndex,
                                                                                    instanceSource, tasksInfo.sizeOfVariables);
                            jacobianLocal(index_source_overall);
                            index_source_overall = IndexTran_Instance2Overall(sourceIndex, max(instanceSource - 1, LLint(0)), tasksInfo.sizeOfVariables);
                            jacobianLocal(index_source_overall);
                        }
                        LLint index_Curr_overall = IndexTran_Instance2Overall(indexCurr, instanceCurr, tasksInfo.sizeOfVariables);
                        jacobianLocal(index_Curr_overall);

                        indexRes++;
                    }
                }
            }

            // x -> x0
            SM_Dynamic j_map = JacobianElimination(tasksInfo, forestInfo);
            EndTimer("SF_H");
            return j_yx * j_map;
        }
    };

    std::unordered_map<JobCEC, std::vector<JobCEC>> GetSensorMapFromSingleJob(
        const TaskSetInfoDerived &tasksInfo, int task_id, TaskSet &precede_tasks, const VectorDynamic &x)
    {
        std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map;

        if (precede_tasks.size() < 2)
        {
            return sensor_map;
        }
        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        LLint totalStartJobs = hyperPeriod / tasks[task_id].period + 1;

        for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
        {
            JobCEC succeed_job = {task_id, (startInstanceIndex)};
            std::vector<JobCEC> precede_jobs;
            double currentJobST = GetStartTime(succeed_job, x, tasksInfo);
            for (auto precede_task : precede_tasks)
            {
                LLint jobIndex = 0;
                while (GetFinishTime({precede_task.id, jobIndex}, x, tasksInfo) <= currentJobST)
                {
                    jobIndex++;
                    if (jobIndex > 10000)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                if (jobIndex > 0)
                {
                    precede_jobs.push_back(JobCEC(precede_task.id, jobIndex - 1));
                }
            }
            if (precede_jobs.size() > 0)
            {
                sensor_map[succeed_job] = precede_jobs;
            }
        }
        return sensor_map;
    }
}