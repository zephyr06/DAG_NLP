#pragma once

#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "Parameters.h"
namespace RTSS21IC_NLP
{

    namespace DAG_SPACE
    {
        using namespace RegularTaskSystem;
        class Prior_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
        {
        public:
            TaskSet tasks;
            vector<LLint> sizeOfVariables;
            int N;
            LLint errorDimension;
            LLint length;
            MAP_Index2Data mapIndex;
            vector<bool> maskForEliminate;
            double priorValue;
            LLint firstTaskIndex;

            Prior_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                                   MAP_Index2Data &mapIndex, vector<bool> &maskForEliminate, double priorValue,
                                   LLint firstTaskIndex,
                                   SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                             tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                             N(tasks.size()), errorDimension(errorDimension),
                                                             maskForEliminate(maskForEliminate), mapIndex(mapIndex),
                                                             priorValue(priorValue), firstTaskIndex(firstTaskIndex)
            {
                length = 0;

                for (int i = 0; i < N; i++)
                {
                    length += sizeOfVariables[i];
                }
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
                res(indexRes++, 0) = (ExtractVariable(startTimeVector, sizeOfVariables, firstTaskIndex, 0) - priorValue) *
                                     weightPrior_factor;

                if (indexRes != errorDimension)
                {
                    cout << Color::red << "The errorDimension is set wrong!" << Color::def << endl;
                    throw;
                }

                return res;
            };
            MatrixDynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
            {
                LLint n0 = 0;
                for (size_t i = 0; i < length; i++)
                    if (maskForEliminate.at(i) == false)
                        n0++;
                if (weightPrior_factor == 0)
                    return GenerateMatrixDynamic(errorDimension, n0);
                else
                {
                    CoutError("no JacobianAnalytic is provided for makespanWeight=0");
                    return GenerateMatrixDynamic(errorDimension, n0);
                }
            }
            Vector evaluateError(const VectorDynamic &startTimeVector,
                                 boost::optional<Matrix &> H = boost::none) const override
            {
                BeginTimer("Prior");
                if (H)
                {
                    if (numericalJaobian)
                        *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                    else
                        *H = JacobianAnalytic(startTimeVector);
                    // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                    if (debugMode == 1)
                    {
                        // cout << "The Jacobian matrix of Prior_ConstraintFactor is " << endl
                        //      << *H << endl;
                    }
                    if (debugMode == 1)
                    {
                        // cout << "The input startTimeVector is " << startTimeVector << endl;
                        cout << "The error vector of Prior is " << Color::blue << f(startTimeVector) << Color::def << endl;
                    }
                }
                EndTimer("Prior");
                return f(startTimeVector);
            }
        };
    }

} // namespace RTSS21IC_NLP
