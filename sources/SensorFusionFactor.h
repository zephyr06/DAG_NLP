#include "DeclareDAG.h"
#include "RegularTasks.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    double ExtractMaxDistance(vector<double> &sourceFinishTime)
    {
        return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
               *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
    }
    class SensorFusion_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        DAG_Model dagTasks;
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        double sensorFusionTol;
        MAP_Index2Data mapIndex;
        vector<bool> maskForEliminate;

        SensorFusion_ConstraintFactor(Key key, DAG_Model &dagTasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                                      double sensorFusionTol, MAP_Index2Data &mapIndex, vector<bool> &maskForEliminate,
                                      SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                                dagTasks(dagTasks),
                                                                tasks(dagTasks.tasks), sizeOfVariables(sizeOfVariables),
                                                                N(tasks.size()), sensorFusionTol(sensorFusionTol),
                                                                errorDimension(errorDimension), mapIndex(mapIndex),
                                                                maskForEliminate(maskForEliminate)
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
        }
        pair<bool, boost::function<VectorDynamic(const VectorDynamic &)>>
        getMappingFunction(int j, VectorDynamic &startTimeVector)
        {
            bool success;
            FuncV2V f;
            return make_pair(false, f);
        }
        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {

            boost::function<Matrix(const VectorDynamic &)> f =
                [this](const VectorDynamic &startTimeVectorOrig)
            {
                VectorDynamic startTimeVector = RecoverStartTimeVector(
                    startTimeVectorOrig, maskForEliminate, mapIndex);
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                // go through all the instances of task 3
                for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
                {
                    const TaskSet &tasksPrev = itr->second;
                    if (tasksPrev.size() > 1)
                    {
                        vector<double> sourceFinishTime;
                        sourceFinishTime.reserve(3);
                        size_t indexCurr = itr->first;

                        for (int instanceCurr = 0; instanceCurr < sizeOfVariables[indexCurr]; instanceCurr++)
                        {
                            sourceFinishTime.clear();

                            // if DAG constraints are violated, addedError will count the violated part
                            double addedError = 0;
                            double startTimeCurr = ExtractVariable(startTimeVector, sizeOfVariables, 3, instanceCurr);

                            // go through three source sensor tasks
                            // for (int sourceIndex = 0; sourceIndex < 3; sourceIndex++)
                            for (size_t sourceIndex = 0; sourceIndex < tasksPrev.size(); sourceIndex++)
                            {
                                LLint instanceSource = floor(startTimeCurr / tasks[sourceIndex].period);
                                // if instanceSource<0, that means startTimeCurr <0, we'll use the first sourceInstance in a period instead
                                // instanceSource = (instanceSource < 0) ? 0 : instanceSource;
                                if (instanceSource < 0)
                                    instanceSource = 0;

                                double startTimeSourceInstance = ExtractVariable(startTimeVector, sizeOfVariables,
                                                                                 sourceIndex, instanceSource);
                                double finishTimeSourceInstance = startTimeSourceInstance + tasks[sourceIndex].executionTime;
                                if (finishTimeSourceInstance <= startTimeCurr)
                                    sourceFinishTime.push_back(finishTimeSourceInstance);
                                else if (instanceSource - 1 >= 0)
                                {
                                    double startTime_k_l_prev = ExtractVariable(startTimeVector, sizeOfVariables,
                                                                                sourceIndex, instanceSource - 1);
                                    if (startTime_k_l_prev + tasks[sourceIndex].executionTime < startTimeCurr)
                                        sourceFinishTime.push_back(startTime_k_l_prev + tasks[sourceIndex].executionTime);
                                    else
                                    {
                                        //in this case, self deadline constraint is violated
                                        sourceFinishTime.push_back(startTime_k_l_prev + tasks[sourceIndex].executionTime);
                                        // cout << "Error in SensorFusion_ConstraintFactor" << endl;
                                        // throw;
                                    }
                                }
                                // there is no previous instance, i.e., DAG constraints are violated,
                                // find a later instance, and return it because it will give a bigger error
                                else
                                {
                                    double startTime_k_l_next = ExtractVariable(startTimeVector,
                                                                                sizeOfVariables, sourceIndex, instanceSource + 1);
                                    sourceFinishTime.push_back(startTime_k_l_next + tasks[sourceIndex].executionTime);
                                    // if all the sources violate DAG constraints, this error will drag them back
                                    addedError += startTime_k_l_next + tasks[sourceIndex].executionTime - startTimeCurr;
                                }
                            }
                            res(indexRes++, 0) = addedError + Barrier(sensorFusionTol - ExtractMaxDistance(sourceFinishTime));
                        }
                    }
                }

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of SensorFusion_ConstraintFactor is " << endl;
                    cout << *H << endl;
                }
                if (debugMode == 1)
                {
                    // cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector of SensorFusion is " << blue << f(startTimeVector) << def << endl;
                }
            }

            return f(startTimeVector);
        }
    };
}