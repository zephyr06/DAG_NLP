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
        std::unordered_map<LLint, LLint> mapIndex_True2Compress;

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
            mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);
        }
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
                                                 ExtractVariable(startTimeVector, sizeOfVariables, i, j) -
                                                 tasks[i].executionTime + 0) *
                                         weightDDL_factor;
                    // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                    res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, i, j) -
                                                 (j * tasks[i].period) + 0) *
                                         weightDDL_factor;
                }
            }

            if (indexRes != errorDimension)
            {
                cout << Color::red << "The errorDimension is set wrong!" << Color::def << endl;
                throw;
            }

            return res;
        };
        MatrixDynamic JacobianAnalytic(VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);

            int m = errorDimension;
            LLint n = length;
            // y -> x
            MatrixDynamic j_yx = GenerateMatrixDynamic(m, n);
            j_yx.resize(m, n);
            // go through m
            LLint index_m = 0;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < int(sizeOfVariables[i]); j++)
                {
                    // Barrier function transforms all the negative error into positive error
                    if ((tasks[i].deadline + j * tasks[i].period -
                         ExtractVariable(startTimeVector, sizeOfVariables, i, j) -
                         tasks[i].executionTime + 0) < 0)
                    {
                        j_yx(index_m * 2, index_m) = 1 * weightDDL_factor;
                    }
                    else
                        j_yx(index_m * 2, index_m) = 0;

                    if ((ExtractVariable(startTimeVector, sizeOfVariables, i, j) -
                         (j * tasks[i].period) + 0) < 0)
                    {
                        j_yx(index_m * 2 + 1, index_m) = -1 * weightDDL_factor;
                    }
                    else
                        j_yx(index_m * 2 + 1, index_m) = 0;

                    index_m++;
                }
            }

            // x -> x0
            MatrixDynamic j_map = JacobianElimination(length, maskForEliminate, n, N,
                                                      sizeOfVariables, mapIndex, mapIndex_True2Compress);
            return j_yx * j_map;
        }

        Vector evaluateError(const VectorDynamic &startTimeVector,
                             boost::optional<Matrix &> H = boost::none) const override
        {

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
                    cout << "The error vector of DDL is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }

            return f(startTimeVector);
        }
    };
}