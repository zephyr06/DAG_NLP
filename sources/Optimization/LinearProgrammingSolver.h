#pragma once
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h" // DebugMode
#include "sources/Utils/testMy.h"     // CoutError

#include "sources/Factors/JacobianAnalyze.h" // GenerateRTDALPOrg

namespace LPOptimizer {

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
// in-place addition, add c to each x_i
inline void VectorAdd(VectorDynamic &x, double c) { x = x + Eigen::MatrixXd::Ones(x.rows(), 1) * c; }

struct CentralVariable {
  CentralVariable() {}
  CentralVariable(VectorDynamic x, VectorDynamic s, VectorDynamic lambda) : x(x), s(s), lambda(lambda) {}

  // inline CentralVariable operator+(const CentralVariable &a, const CentralVariable &b)
  // {
  //     CentralVariable res = a;
  //     res.x = res.x + b.x;
  //     res.s = res.s + b.s;
  //     res.lambda = res.lambda + b.lambda;
  //     return res;
  // }
  // inline CentralVariable operator-(const CentralVariable &a, const CentralVariable &b)
  // {
  //     CentralVariable res = a;
  //     res.x = res.x - b.x;
  //     res.s = res.s - b.s;
  //     res.lambda = res.lambda - b.lambda;
  //     return res;
  // }

  inline CentralVariable &operator+(const CentralVariable &a) {
    x = x + a.x;
    s = s + a.s;
    lambda = lambda + a.lambda;
    return *this;
  }

  inline CentralVariable &operator-(const CentralVariable &a) {
    x = x - a.x;
    s = s - a.s;
    lambda = lambda - a.lambda;
    return *this;
  }

  VectorDynamic x;
  VectorDynamic s;
  VectorDynamic lambda;
};

class LPData {
public:
  Eigen::SparseMatrix<double> A_;
  VectorDynamic b_;
  VectorDynamic c_;
  size_t m_;
  size_t n_;
  CentralVariable centralVarCurr_;

  LPData() {}

  LPData(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c);

  // TODO: remove all the matrix inverse
  CentralVariable GenerateInitialLP();

  CentralVariable SolveLinearSystem();

  void ApplyCentralDelta(const CentralVariable &centralDelta, double eta = 0.7);

  inline double Duality() const {
    return (centralVarCurr_.x.transpose() * centralVarCurr_.s / centralVarCurr_.s.rows())(0, 0);
  }

  void print() const {
    std::cout << "\nA: \n" << Eigen::MatrixXd(A_) << std::endl;
    std::cout << "\nb: \n" << b_ << std::endl;
    std::cout << "\nc: \n" << c_ << std::endl;
  }
};

// Solve the following LP:
// min_x c^T x
// subject to Ax <= b
// This algorithm is based on primal-dual interior-point method, following the tutorial in
// Nocedal07Numerical_Optimization During the optimization process, this algorithm doesn't perform matrix
// permutation
// TODO: avoid more data copy/paste
VectorDynamic SolveLP(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c,
                      double precision = GlobalVariablesDAGOpt::NumericalPrecision);

VectorDynamic SolveLP(LPData &lpData, double precision = GlobalVariablesDAGOpt::NumericalPrecision);

} // namespace LPOptimizer

namespace OrderOptDAG_SPACE {
using namespace LPOptimizer;

LPData GenerateRTDALPOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                         const std::vector<uint> processorJobVec, int processorNum,
                         bool lessJobOrderConstraints = false);

void RoundIPMResults(VectorDynamic &startTimeVector, double precision = 1e-4);

VectorDynamic OptRTDA_IPMOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                             SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum,
                             bool lessJobOrderConstraints = false);
} // namespace OrderOptDAG_SPACE
