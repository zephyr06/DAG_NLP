#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "EliminationForest_utils.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    class DAG_ConstraintFactor : public BaseSchedulingFactor
    {
    public:
        DAG_Model dagTasks;

        DAG_ConstraintFactor(Key key, DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, EliminationForest &forestInfo,
                             LLint errorDimension,
                             SharedNoiseModel model) : BaseSchedulingFactor(key, tasksInfo, forestInfo, errorDimension, model),
                                                       dagTasks(dagTasks)

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

        MatrixDynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);

            int m = errorDimension;
            LLint n = length;
            // y -> x
            SM_Dynamic j_yx(m, n);
            // go through m
            LLint index_m = 0;

            // add dependency constraints
            // !! Make sure unordered_map go through edges following the same order as did in evaluateError
            for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
            {
                const TaskSet &tasksPrev = itr->second;
                size_t indexNext = itr->first;
                for (size_t i = 0; i < tasksPrev.size(); i++)
                {
                    double err = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0) -
                                 ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0) -
                                 tasksPrev[i].executionTime;
                    // err = Barrier(err);
                    if (err >= deltaOptimizer)
                    {
                        index_m++;
                        // continue;
                    }
                    else if (err <= -deltaOptimizer)
                    {
                        LLint index_N_overall = IndexTran_Instance2Overall(indexNext, 0, sizeOfVariables);
                        LLint index_P_overall = IndexTran_Instance2Overall(tasksPrev[i].id, 0, sizeOfVariables);
                        // j_yx(index_m, index_N_overall) = -1;
                        UpdateSM(-1, index_m, index_N_overall, j_yx);
                        UpdateSM(1, index_m++, index_P_overall, j_yx);

                        // j_yx(index_m++, index_P_overall) = 1;
                    }
                    else if (err > -deltaOptimizer && err < deltaOptimizer)
                    {
                        double x1 = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0);
                        double x2 = ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0);
                        double c = tasksPrev[i].executionTime;
                        double errPlus1 = Barrier(x1 + deltaOptimizer - x2 - c);
                        double errMinus1 = Barrier(x1 - deltaOptimizer - x2 - c);
                        double errPlus2 = Barrier(x1 - deltaOptimizer - x2 - c);
                        double errMinus2 = Barrier(x1 + deltaOptimizer - x2 - c);
                        LLint index_N_overall = IndexTran_Instance2Overall(indexNext, 0, sizeOfVariables);
                        LLint index_P_overall = IndexTran_Instance2Overall(tasksPrev[i].id, 0, sizeOfVariables);
                        // j_yx(index_m, index_N_overall) = (errPlus1 - errMinus1) / 2 / deltaOptimizer;
                        UpdateSM((errPlus1 - errMinus1) / 2 / deltaOptimizer, index_m, index_N_overall, j_yx);
                        // j_yx(index_m++, index_P_overall) = (errPlus2 - errMinus2) / 2 / deltaOptimizer;
                        UpdateSM((errPlus2 - errMinus2) / 2 / deltaOptimizer, index_m++, index_P_overall, j_yx);
                    }
                }
            }

            // x -> x0
            SM_Dynamic j_map = JacobianElimination(tasksInfo, forestInfo);

            return j_yx * j_map;
        }
        Vector evaluateError(const VectorDynamic &startTimeVector,
                             boost::optional<Matrix &> H = boost::none) const override
        {
            BeginTimer("DAG");
            if (H)
            {
                if (numericalJaobian)
                    *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                else
                    *H = JacobianAnalytic(startTimeVector);
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
            EndTimer("DAG");
            return f(startTimeVector);
        }
    };

}