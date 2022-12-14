#include "sources/Optimization/LPSolverCplex.h"
#include "sources/Utils/profilier.h"

namespace LPOptimizer
{

    VectorDynamic SolveLP_Cplex(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c)
    {
        BeginTimer("SolveLP_Cplex");
        int numVariables_ = A.cols();

        IloEnv env_ = IloEnv();
        IloModel model_ = IloModel(env_);
        IloCplex cplexSolver_ = IloCplex(env_);
        cplexSolver_.setOut(env_.getNullStream());

        // add variables
        IloNumVarArray varArray_ = IloNumVarArray(env_, numVariables_, 0, 1e4, IloNumVar::Float); // variable range: 0 - hyperPeriod
        EndTimer("SolveLP_Cplex");
        // add constraints
        Eigen::MatrixXd Adense = Eigen::MatrixXd(A);
        for (uint i = 0; i < Adense.rows(); i++)
        {
            IloNumExpr row(env_);
            for (uint j = 0; j < Adense.cols(); ++j)
            {
                if (Adense(i, j))
                    row += Adense(i, j) * varArray_[j];
            }

            model_.add(row <= b(i));
            row.end();
        }

        BeginTimer("SolveLP_Cplex");
        // add obj
        {
            IloNumExpr row(env_);
            for (uint j = 0; j < c.rows(); ++j)
            {
                if (c(j))
                    row += c(j) * varArray_[j];
            }
            model_.add(IloMinimize(env_, row));
            row.end();
        }

        cplexSolver_.extract(model_);
        bool found_feasible_solution = cplexSolver_.solve();

        IloNumArray values_optimized(env_, numVariables_);
        if (found_feasible_solution)
        {
            auto status = cplexSolver_.getStatus();
            cplexSolver_.getValues(varArray_, values_optimized);
            if (GlobalVariablesDAGOpt::debugMode)
            {
                std::cout << "Values are :" << values_optimized << "\n";
                std::cout << status << " solution found: " << cplexSolver_.getObjValue() << "\n";
            }
        }

        VectorDynamic optimizedStartTimeVector_(numVariables_);
        for (int i = 0; i < numVariables_; i++)
        {
            optimizedStartTimeVector_(i) = values_optimized[i];
        }

        // release memory
        cplexSolver_.end();
        varArray_.end();
        model_.end();
        env_.end();
        EndTimer("SolveLP_Cplex");
        return optimizedStartTimeVector_;
    }
} // namespace LPOptimizer