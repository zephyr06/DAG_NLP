#include "BaseSchedulingFactor.h"
namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    class DDL_ConstraintFactor : public BaseSchedulingFactor
    {
    public:
        DDL_ConstraintFactor(Key key, TaskSetInfoDerived &tasksInfo,
                             EliminationForest &forestInfo,
                             LLint errorDimension,
                             SharedNoiseModel model) : BaseSchedulingFactor(key, tasksInfo,
                                                                            forestInfo, errorDimension, model)
        {
        }
        boost::function<MatrixRowMajor(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            // self DDL
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
                {
                    // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
                    res(indexRes++, 0) = Barrier(tasksInfo.tasks[i].deadline + j * tasksInfo.tasks[i].period -
                                                 ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                                 tasksInfo.tasks[i].executionTime + 0) *
                                         weightDDL_factor;
                    // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                    res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                                 (j * tasksInfo.tasks[i].period) + 0) *
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
        SM_Dynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            // BeginTimer("DDL_H");
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, forestInfo);

            int m = errorDimension;
            LLint n = tasksInfo.length;
            // y -> x
            SM_Dynamic j_yx(m, n);
            j_yx.resize(m, n);
            // go through m
            LLint index_m = 0;
            // Barrier function transforms all the negative error into positive error
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
                {
                    // if (index_m == 99)
                    // {
                    //     int a = 1;
                    // }
                    // finish time is smaller than deadline
                    double err1 = tasksInfo.tasks[i].deadline + j * tasksInfo.tasks[i].period -
                                  ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                  tasksInfo.tasks[i].executionTime;

                    if (err1 <= 0 - deltaOptimizer)
                    {
                        j_yx.insert(index_m * 2, index_m) = 1 * weightDDL_factor;
                    }
                    else if (err1 <= deltaOptimizer)
                    {

                        double errP1 = Barrier(err1 - deltaOptimizer);
                        double errM1 = Barrier(err1 + deltaOptimizer);
                        j_yx.insert(index_m * 2, index_m) = (errP1 - errM1) / 2 / deltaOptimizer * weightDDL_factor;
                    }
                    else
                        j_yx.insert(index_m * 2, index_m) = 0;

                    // start time is larger than start of period
                    double err2 = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, i, j) -
                                  (j * tasksInfo.tasks[i].period);
                    if (err2 <= 0 - deltaOptimizer)
                    {
                        j_yx.insert(index_m * 2 + 1, index_m) = -1 * weightDDL_factor;
                    }
                    else if (err2 <= deltaOptimizer)
                    {
                        double errP1 = Barrier(err2 + deltaOptimizer);
                        double errM1 = Barrier(err2 - deltaOptimizer);
                        j_yx.insert(index_m * 2 + 1, index_m) = (errP1 - errM1) / 2 / deltaOptimizer * weightDDL_factor;
                    }
                    else
                        j_yx.insert(index_m * 2 + 1, index_m) = 0;

                    index_m++;
                }
            }

            // x -> x0
            SM_Dynamic j_map = JacobianElimination(tasksInfo, forestInfo);
            // cout << j_map << endl
            //      << endl;
            // EndTimer("DDL_H");
            return j_yx * j_map;
        }

        Vector evaluateError(const VectorDynamic &startTimeVector,
                             boost::optional<Matrix &> H = boost::none) const override
        {
            BeginTimer("DDL_All");
            if (H)
            {

                if (numericalJaobian)
                    *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                else
                    *H = JacobianAnalytic(startTimeVector);

                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    // cout << "The Jacobian matrix of DDL_ConstraintFactor is " << endl
                    //      << *H << endl;
                }
                if (debugMode == 1)
                {
                    // cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector of DDL is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }
            EndTimer("DDL_All");
            return f(startTimeVector);
        }
    };
}