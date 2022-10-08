#ifndef OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#define OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#include <iostream>
#include <memory>
#include "sources/Utils/BatchUtils.h"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

namespace DAG_SPACE
{
    class ScheduleOptimizer
    {
    public:
        ScheduleOptimizer()
        {
            p_env_ = std::shared_ptr<IloEnv>(new IloEnv);
            p_model_ = std::shared_ptr<IloModel>(new IloModel(*p_env_));
            p_cplex_solver_ = std::shared_ptr<IloCplex>(new IloCplex(*p_env_));
        }
        void reset()
        {
            p_env_->end();
            p_model_->end();
            p_cplex_solver_->clear();
            p_env_ = std::shared_ptr<IloEnv>(new IloEnv);
            p_model_ = std::shared_ptr<IloModel>(new IloModel(*p_env_));
            p_cplex_solver_ = std::shared_ptr<IloCplex>(new IloCplex(*p_env_));
        }

        inline std::shared_ptr<IloEnv> getEnv()
        {
            return p_env_;
        }
        inline std::shared_ptr<IloModel> getModel()
        {
            return p_model_;
        }
        inline std::shared_ptr<IloCplex> getSolver()
        {
            return p_cplex_solver_;
        }
        void setScheduleResult(ScheduleResult &res)
        {
            result_to_optimize_ = res;
        }
        ScheduleResult getOptimizedResult()
        {
            return result_after_optimization_;
        }

        void print()
        {
            std::cout << "ScheduleOptimizer successfully print something!!!\n";
        }
        void SolveLP()
        {
            IloEnv env = *getEnv();
            IloCplex cplex = *getSolver();
            IloModel model = *getModel();

            IloNumVar var1(env);
            IloNumVarArray var_array(env, 2, -10, 200, IloNumVar::Float);
            var_array[0].setBounds(10.0, 20.0);
            var_array[1].setBounds(15.0, 20.0);

            model.add(IloRange(env, var_array[0] + var_array[1], 35));
            model.add(30 <= var_array[0] + var_array[1]);

            IloExpr myexpr(env);
            myexpr -= var_array[0];
            myexpr -= var_array[1];
            model.add(IloRange(env, myexpr, 0));
            myexpr.end();

            IloRangeArray constraint_array(env);
            constraint_array.add(IloRange(env, 0, 100));
            constraint_array.add(IloRange(env, 10, 200));
            constraint_array[0].setLinearCoef(var_array[0], 1);
            IloNumArray coeffs(env, 2, 1, 1);
            std::cout << coeffs.getSize() << "\n";
            constraint_array[1].setLinearCoefs(var_array, coeffs);
            model.add(constraint_array);
            // std::cout << constraint_array.getSize()<<"\n";

            model.add(var1 >= var_array[0] * 2 + 0.5 * var_array[1]);
            model.add(var1 >= var_array[0] * 0.9 + 1.8 * var_array[1]);

            // model.add(IloMinimize(env, var_array[0] * 2 + 0.5 * var_array[1]));
            model.add(IloMinimize(env, var1));

            cplex.extract(model);
            cplex.solve();

            auto status = cplex.getStatus();
            std::cout << cplex.getValue(var1) << "\n";
            std::cout << cplex.getValue(var_array[0]) << ",  " << cplex.getValue(var_array[1]) << "\n";
            std::cout << status << " solution found: " << cplex.getObjValue() << "\n";

            cplex.clear();
            env.end();
        }

    private:
        std::shared_ptr<IloEnv> p_env_;
        std::shared_ptr<IloModel> p_model_;
        std::shared_ptr<IloCplex> p_cplex_solver_;
        ScheduleResult result_to_optimize_;
        ScheduleResult result_after_optimization_;
        TaskSetInfoDerived taskInfo_;
    };
} // namespace DAG_SPACE
#endif // OPTIMIZATION_SCHEDULE_OPTIMIZER_H_