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
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        double sensorFusionTol;
        MAP_Index2Data mapIndex;
        vector<bool> maskForEliminate;

        SensorFusion_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                                      double sensorFusionTol, MAP_Index2Data &mapIndex, vector<bool> &maskForEliminate,
                                      SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                                tasks(tasks), sizeOfVariables(sizeOfVariables),
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
                cout << "The startTimeVector: " << startTimeVector.transpose() << endl;
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                vector<double> sourceFinishTime;
                // TODO: this is a customized number
                sourceFinishTime.reserve(3);
                // go through all the instances of task 3
                for (int instanceCurr = 0; instanceCurr < sizeOfVariables[3]; instanceCurr++)
                {
                    double startTimeCurr = ExtractVariable(startTimeVector, sizeOfVariables, 3, instanceCurr);
                    // go through three source sensor tasks
                    for (int sourceIndex = 0; sourceIndex < 3; sourceIndex++)
                    {
                        LLint instanceSource = floor(startTimeCurr / tasks[sourceIndex].period);
                        double startTimeSourceInstance = ExtractVariable(startTimeVector, sizeOfVariables, sourceIndex, instanceSource);
                        double finishTimeSourceInstance = startTimeSourceInstance + tasks[sourceIndex].executionTime;
                        if (finishTimeSourceInstance < startTimeCurr)
                            sourceFinishTime.push_back(finishTimeSourceInstance);
                        else if (instanceSource - 1 >= 0)
                        {
                            double startTime_k_l_prev = ExtractVariable(startTimeVector, sizeOfVariables, sourceIndex, instanceSource - 1);
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
                            double startTime_k_l_next = ExtractVariable(startTimeVector, sizeOfVariables, sourceIndex, instanceSource + 1);
                            sourceFinishTime.push_back(startTime_k_l_next + tasks[sourceIndex].executionTime);
                        }
                    }
                    res(indexRes++, 0) = Barrier(sensorFusionTol - ExtractMaxDistance(sourceFinishTime));
                }

                // res(0, 0) = BarrierLog(sensorFusionTol - ExtractMaxDistance(sourceFinishTime));

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of SensorFusion_ConstraintFactor is " << endl
                         << *H << endl;
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