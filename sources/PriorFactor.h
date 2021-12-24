#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "Parameters.h"
#include "BaseSchedulingFactor.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class Prior_ConstraintFactor : public BaseSchedulingFactor
    {
    public:
        double priorValue;
        LLint firstTaskIndex;

        Prior_ConstraintFactor(Key key, TaskSetInfoDerived &tasksInfo, EliminationForest &forestInfo,
                               LLint errorDimension, double priorValue,
                               LLint firstTaskIndex,
                               SharedNoiseModel model) : BaseSchedulingFactor(key, tasksInfo, forestInfo, errorDimension, model),
                                                         priorValue(priorValue), firstTaskIndex(firstTaskIndex)
        {
        }
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            // self DDL
            res(indexRes++, 0) = (ExtractVariable(startTimeVector,
                                                  tasksInfo.sizeOfVariables, firstTaskIndex, 0) -
                                  priorValue) *
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
            for (LLint i = 0; i < tasksInfo.length; i++)
                if (forestInfo.maskForEliminate.at(i) == false)
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