#include "DeclareDAG.h"
#include "RegularTasks.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class DDL_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        MAP_Index2Data mapIndex;
        vector<bool> maskForEliminate;

        DDL_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                             MAP_Index2Data &mapIndex, vector<bool> &maskForEliminate,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size()), errorDimension(errorDimension),
                                                       maskForEliminate(maskForEliminate), mapIndex(mapIndex)
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
        }

        /**
         * @brief Get the Mapping Function object
         *          This function is skipped because we don't consider its related elimination for now
         * 
         * @param j 
         * @param startTimeVector 
         * @return pair<bool, boost::function<VectorDynamic(const VectorDynamic &)>> 
         */
        pair<bool, boost::function<VectorDynamic(const VectorDynamic &)>>
        getMappingFunction(int j, VectorDynamic &startTimeVector)
        {
            bool success;
            FuncV2V f;
            return make_pair(false, f);
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

                // self DDL
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < int(sizeOfVariables[i]); j++)
                    {
                        // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
                        res(indexRes++, 0) = Barrier(tasks[i].deadline + j * tasks[i].period -
                                                     ExtractVariable(startTimeVector, sizeOfVariables, i, j) - tasks[i].executionTime + 0);
                        // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                        res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, i, j) - (j * tasks[i].period) + 0);
                    }
                }

                if (indexRes != errorDimension)
                {
                    cout << red << "The errorDimension is set wrong!" << def << endl;
                    throw;
                }

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DDL_ConstraintFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    // cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector of DDL is " << blue << f(startTimeVector) << def << endl;
                }
            }

            return f(startTimeVector);
        }
    };
}