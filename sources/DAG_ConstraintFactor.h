#include "DeclareDAG.h"
#include "RegularTasks.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class DAG_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        vector<bool> maskForEliminate;
        MAP_Index2Data mapIndex;

        DAG_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                             MAP_Index2Data &mapIndex, const vector<bool> &maskForEliminate,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size()), errorDimension(errorDimension),
                                                       maskForEliminate(maskForEliminate),
                                                       mapIndex(mapIndex)
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
        }

        Vector evaluateError(const VectorDynamic &startTimeVector,
                             boost::optional<Matrix &> H = boost::none) const override
        {

            boost::function<Matrix(const VectorDynamic &)> f =
                [this](const VectorDynamic &startTimeVectorOrig)
            {
                VectorDynamic startTimeVector = RecoverStartTimeVector(
                    startTimeVectorOrig, maskForEliminate, mapIndex);
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                // dependency, self DDL, sensor fusion, event chain
                // minimize makespan
                res(indexRes++, 0) = BarrierLog(ExtractVariable(startTimeVector, sizeOfVariables, 4, 0) -
                                                ExtractVariable(startTimeVector, sizeOfVariables, 0, 0) + 0) *
                                     makespanWeight;

                // add dependency constraints
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 0, 0) - tasks[0].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 1, 0) - tasks[1].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 2, 0) - tasks[2].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 4, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) - tasks[3].executionTime + 0);

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DAG_ConstraintFactor (including makespac) is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    // cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector of DAG is " << blue << f(startTimeVector) << def << endl;
                }
            }

            return f(startTimeVector);
        }
    };

}