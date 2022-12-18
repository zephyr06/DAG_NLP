
#pragma once
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"

namespace LPOptimizer
{
    VectorDynamic SolveLP_Cplex(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c);
} // namespace LPOptimizer