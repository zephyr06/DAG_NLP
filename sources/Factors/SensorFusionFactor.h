#include "sources/Utils/DeclareDAG.h"
#include "sources/TaskModel/RegularTasks.h"
#include "sources/Optimization/EliminationForest_utils.h"
#include "sources/Factors/BaseSchedulingFactor.h"
namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
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
                    *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
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

    void recordText()
    {
        ;
        // sourceFinishTime.clear();

        // // if DAG constraints are violated, addedErrorDAG will count the violated part
        // double addedErrorDAG = 0;
        // double addedErrorDDL = 0;
        // double startTimeCurr = ExtractVariable(startTimeVector, sizeOfVariables, indexCurr, instanceCurr);
        // LLint index_Curr_overall = IndexTran_Instance2Overall(indexCurr, instanceCurr, sizeOfVariables);

        // // go through three source sensor tasks in the example
        // for (size_t ii = 0; ii < tasksPrev.size(); ii++)
        // {
        //     int sourceIndex = tasksPrev.at(ii).id;

        //     LLint instanceSource = floor(startTimeCurr / tasks[sourceIndex].period);
        //     // if instanceSource<0, that means startTimeCurr <0, we'll use the first sourceInstance in a period instead
        //     // instanceSource = (instanceSource < 0) ? 0 : instanceSource;
        //     if (instanceSource < 0)
        //         instanceSource = 0;
        //     else if (instanceSource > sizeOfVariables[sourceIndex] - 1)
        //         instanceSource = sizeOfVariables[sourceIndex] - 1;
        //     LLint index_source_overall = IndexTran_Instance2Overall(sourceIndex, instanceSource, sizeOfVariables);

        //     double startTimeSourceInstance = ExtractVariable(startTimeVector, sizeOfVariables,
        //                                                      sourceIndex, instanceSource);
        //     double finishTimeSourceInstance = startTimeSourceInstance + tasks[sourceIndex].executionTime;
        //     if (finishTimeSourceInstance <= startTimeCurr)
        //     {
        //         sourceFinishTime.push_back(IndexData{index_source_overall, finishTimeSourceInstance});
        //     }
        //     else if (instanceSource - 1 >= 0)
        //     {
        //         double startTime_k_l_prev = ExtractVariable(startTimeVector, sizeOfVariables,
        //                                                     sourceIndex, instanceSource - 1);
        //         LLint index_source_prev_overall = IndexTran_Instance2Overall(sourceIndex, instanceSource - 1, sizeOfVariables);
        //         if (startTime_k_l_prev + tasks[sourceIndex].executionTime >= startTimeCurr)
        //         {
        //             //in this case, self deadline constraint is violated, and so we'll just add addedErrorDDL
        //             // addedErrorDDL += startTime_k_l_prev + tasks[sourceIndex].executionTime - startTimeCurr;
        //             // negative error will become positive
        //             UpdateSM(-1, indexRes, index_source_prev_overall, j_yx);
        //             UpdateSM(1, indexRes, index_Curr_overall, j_yx);
        //         }
        //         sourceFinishTime.push_back(IndexData{index_source_prev_overall, startTime_k_l_prev + tasks[sourceIndex].executionTime});
        //     }
        //     // there is no previous instance, i.e., DAG constraints are violated,
        //     // instanceSource = 0
        //     // find a later instance, and return it because it will give a bigger error
        //     else
        //     {
        //         double startTime_k_l_next = startTimeSourceInstance;
        //         LLint index_source_prev_overall = index_source_overall;
        //         sourceFinishTime.push_back(IndexData{index_source_prev_overall, startTime_k_l_next});
        //         // if all the sources violate DAG constraints, this error will drag them back
        //         if (withAddedSensorFusionError)
        //         {
        //             // if (addedErrorDAG == 0)
        //             // {
        //             // addedErrorDAG = startTime_k_l_next + tasks[sourceIndex].executionTime - startTimeCurr;
        //             UpdateSM(-1, indexRes, index_source_overall, j_yx);
        //             UpdateSM(1, indexRes, index_Curr_overall, j_yx);
        //             // }
        //             // else
        //             //     addedErrorDAG = min(addedErrorDAG, startTime_k_l_next +
        //             //                                            tasks[sourceIndex].executionTime - startTimeCurr);
        //         }
        //     }
        // }
        // // this error is not well tested yet
        // // double sensorFreshError = Barrier(FreshTol - (startTimeCurr +
        // //                                               tasks[indexCurr].executionTime -
        // //                                               *min_element(sourceFinishTime.begin(),
        // //                                                            sourceFinishTime.end())));

        // // res(indexRes, 0) = addedErrorDDL + addedErrorDAG +
        // //                    Barrier(sensorFusionTolerance - ExtractMaxDistance(sourceFinishTime)) +
        // //                    sensorFreshError;
        // auto sth = ExtractMaxDistance(sourceFinishTime);
        // int maxIndex = sth.first;
        // int minIndex = sth.second;
        // double maxDiff = sourceFinishTime[maxIndex].time - sourceFinishTime[minIndex].time;
        // if (sensorFusionTolerance - maxDiff < 0)
        // {
        //     UpdateSM(1, indexRes, sourceFinishTime[maxIndex].index, j_yx);
        //     UpdateSM(-1, indexRes, sourceFinishTime[minIndex].index, j_yx);
        // }
    }
}