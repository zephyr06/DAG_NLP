#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "GraphUtilsFromBGL.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class MakeSpanFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        DAG_Model dagTasks;
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        vector<bool> maskForEliminate;
        MAP_Index2Data mapIndex;
        int sinkNode;

        MakeSpanFactor(Key key, DAG_Model &dagTasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                       MAP_Index2Data &mapIndex, const vector<bool> &maskForEliminate,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 dagTasks(dagTasks),
                                                 tasks(dagTasks.tasks), sizeOfVariables(sizeOfVariables),
                                                 N(tasks.size()), errorDimension(errorDimension),
                                                 maskForEliminate(maskForEliminate),
                                                 mapIndex(mapIndex)
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
            sinkNode = FindSinkNode(dagTasks);
        }
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            // minimize makespan
            double finishTimeDAG = ExtractVariable(startTimeVector, sizeOfVariables, sinkNode,
                                                   sizeOfVariables[sinkNode] - 1) +
                                   tasks[sinkNode].executionTime;
            double startTimeDAG = startTimeVector.minCoeff();
            // res(indexRes++, 0) = BarrierLog(finishTimeDAG - startTimeDAG) *
            //                      makespanWeight;
            res(indexRes++, 0) = (finishTimeDAG - startTimeDAG) *
                                 makespanWeight;

            return res;
        };
        MatrixDynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            LLint n0 = 0;
            for (size_t i = 0; i < length; i++)
                if (maskForEliminate.at(i) == false)
                    n0++;
            if (makespanWeight == 0)
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
            BeginTimer("MakeSpan");
            if (H)
            {
                if (numericalJaobian)
                    *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                else
                    *H = JacobianAnalytic(startTimeVector);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of MakeSpanFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    // cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector of MakeSpanFactor is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }
            EndTimer("MakeSpan");
            return f(startTimeVector);
        }
    };

}