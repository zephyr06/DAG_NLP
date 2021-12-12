#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "GraphUtilsFromBGL.h"

/**
 * @brief 
 * 
 * @param x for smallest, smallSecond
 * @param y for largest, largeSecond
 * @param smallest 
 * @param smallSecond 
 * @param largest 
 * @param largeSecond 
 */
void FindLargeSmall(const VectorDynamic &x, const VectorDynamic &y,
                    LLint &smallest, LLint &smallSecond,
                    LLint &largest, LLint &largeSecond)
{
    LLint length = x.rows();
    if (length < 4)
    {
        CoutError("Input to FindLargeSmall must have at least four elements!");
    }
    double v_se = INT_MAX;
    double v_ss = INT_MAX;
    double v_le = INT_MIN;
    double v_ls = INT_MIN;
    for (int i = 0; i < length; i++)
    {
        if (y.coeffRef(i, 0) >= v_le)
        {
            largeSecond = largest;
            largest = i;
            v_ls = v_le;
            v_le = y.coeff(i, 0);
        }
        else if (y.coeffRef(i, 0) > v_ls)
        {
            largeSecond = i;
            v_ls = y.coeff(i, 0);
        }

        if (x.coeffRef(i, 0) <= v_se)
        {
            smallSecond = smallest;
            smallest = i;
            v_ss = v_se;
            v_se = x.coeff(i, 0);
        }
        else if (x.coeffRef(i, 0) < v_ss)
        {
            smallSecond = i;
            v_ss = x.coeff(i, 0);
        }
    }
    return;
}

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
        LLint lengthCompressed;
        int sinkNode;
        std::unordered_map<LLint, LLint> mapIndex_True2Compress;

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
            lengthCompressed = 0;
            for (LLint i = 0; i < length; i++)
            {
                if (maskForEliminate[i] == false)
                    lengthCompressed++;
            }
            mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);
        }

        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVector)
        {
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;
            LLint smallest = 0;
            LLint smallSecond = 0;
            LLint largest = 0;
            LLint largeSecond = 0;
            // minimize makespan
            VectorDynamic finishTimeVector = FindFinishTime(startTimeVector);
            FindLargeSmall(startTimeVector, finishTimeVector, smallest, smallSecond, largest, largeSecond);
            double startTimeDAG = startTimeVector.minCoeff();
            res(indexRes++, 0) = (finishTimeVector(largest, 0) - startTimeVector(smallest, 0)) *
                                 makespanWeight;

            return res;
        };
        /**
         * @brief 
         * 
         * @param startTimeVectorOrig compressed startTimeVector!!!
         * @return VectorDynamic finish time vector after compression
         */
        VectorDynamic FindFinishTime(const VectorDynamic &startTimeVectorOrig) const
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                                   maskForEliminate, mapIndex);
            VectorDynamic finishTimeVector = startTimeVector;
            int m = startTimeVector.rows();
            for (int i = 0; i < m; i++)
            {
                LLint taskId = BigIndex2TaskIndex(i, sizeOfVariables);
                finishTimeVector(i, 0) += tasks[taskId].executionTime;
            }
            VectorDynamic finishTimeVectorOrig = startTimeVectorOrig;
            return CompresStartTimeVector(finishTimeVector, maskForEliminate.size(), maskForEliminate);
        }
        /**
         * @brief This function assumes errorDimension=1!
         * 
         * @param startTimeVectorOrig 
         * @return SM_Dynamic 
         */
        SM_Dynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);
            // y -> x
            SM_Dynamic j_yx(errorDimension, length);

            LLint indexRes = 0;
            LLint smallest = 0;
            LLint smallSecond = 0;
            LLint largest = 0;
            LLint largeSecond = 0;
            // minimize makespan
            VectorDynamic finishTimeVector = FindFinishTime(startTimeVectorOrig);
            FindLargeSmall(startTimeVector, finishTimeVector, smallest, smallSecond, largest, largeSecond);
            // numerical method for jacobian
            auto NumericalFunc = [&](LLint index)
            {
                startTimeVector(index, 0) += deltaOptimizer;
                double resPlus = f(startTimeVector)(0, 0);
                startTimeVector(index, 0) -= 2 * deltaOptimizer;
                double resMinus = f(startTimeVector)(0, 0);
                startTimeVector(index, 0) += deltaOptimizer;
                j_yx.coeffRef(0, index) = (resPlus - resMinus) / 2 / deltaOptimizer;
            };
            NumericalFunc(smallest);
            NumericalFunc(smallSecond);
            NumericalFunc(largest);
            NumericalFunc(largeSecond);

            SM_Dynamic j_map = JacobianElimination(length, lengthCompressed,
                                                   sizeOfVariables, mapIndex, mapIndex_True2Compress);
            return j_yx * j_map;
        }
        Vector evaluateError(const VectorDynamic &startTimeVectorOrig,
                             boost::optional<Matrix &> H = boost::none) const override
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);
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
                    cout << Color::green << "The error vector of MakeSpanFactor is " << f(startTimeVector) << Color::def << endl;
                }
            }
            EndTimer("MakeSpan");
            return f(startTimeVector);
        }
    };

}