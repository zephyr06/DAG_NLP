#include "DeclareDAG.h"
#include "RegularTasks.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class DAG_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
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

        DAG_ConstraintFactor(Key key, DAG_Model &dagTasks, vector<LLint> sizeOfVariables, LLint errorDimension,
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
        }
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            // dependency, self DDL, sensor fusion, event chain

            // add dependency constraints
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                const TaskSet &tasksPrev = itr->second;
                size_t indexNext = itr->first;
                for (size_t i = 0; i < tasksPrev.size(); i++)
                {
                    double err = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0) -
                                 ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0) -
                                 tasksPrev[i].executionTime;
                    res(indexRes++, 0) = Barrier(err);
                }
            }
            return res;
        };
        Vector evaluateError(const VectorDynamic &startTimeVector,
                             boost::optional<Matrix &> H = boost::none) const override
        {

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
                    cout << "The error vector of DAG is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }

            return f(startTimeVector);
        }
    };

}