#include "sources/Optimization/LinearProgrammingSolver.h"
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <Eigen/SparseQR>
// TODO: if there is time, consider using band matrix from eigen
namespace LPOptimizer {
LPData::LPData(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const SpVec &c)
    : b_(b), m_(A.rows()), n_(A.cols()) {
  if (m_ > n_) {
    // A_.resize(m_, n_ + m_);
    A_ = Eigen::SparseMatrix<double>(m_, m_ + n_);
    A_.reserve(A.nonZeros() + m_);

    for (uint c = 0; c < A.cols(); ++c) {
      A_.startVec(c); // Important: Must be called once for each column before inserting!
      for (Eigen::SparseMatrix<double>::InnerIterator itL(A, c); itL; ++itL)
        A_.insertBack(itL.row(), c) = itL.value();
    }
    A_.finalize();
    for (uint i = 0; i < m_; i++)
      A_.insert(i, i + n_) = 1;

    c_ = SpVec(m_ + n_);
    c_.setZero();
    for (int k = 0; k < c.outerSize(); k++)
      for (Eigen::SparseVector<double>::InnerIterator it(c, k); it; ++it) {
        c_.insert(it.row()) = it.value();
      }

    // c_ << c, zeros;
  } else {
    CoutError("A's dimension is wrong in LPData!");
  }
  centralVarCurr_ = GenerateInitialLP();
}

CentralVariable LPData::GenerateInitialLP() {
  Eigen::SparseMatrix<double> AA = A_ * A_.transpose();
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> AAFact(AA);
  // auto AAFact = solver.compute(AA);
  VectorDynamic xCurr_ = A_.transpose() * (AAFact.solve(b_));
  VectorAdd(xCurr_, std::max(-1.5 * xCurr_.minCoeff(), 0.0));

  VectorDynamic lambdaCurr_ = AAFact.solve(A_ * c_);
  VectorDynamic sCurr_ = c_ - A_.transpose() * lambdaCurr_;
  VectorAdd(sCurr_, std::max(-1.5 * sCurr_.minCoeff(), 0.0));
  double deltax = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / sCurr_.sum() / 2.0;
  double deltas = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / xCurr_.sum() / 2.0;
  VectorAdd(xCurr_, deltax);
  VectorAdd(sCurr_, deltas);

  return CentralVariable{xCurr_, sCurr_, lambdaCurr_};
}

double GetAlphaAff(const VectorDynamic &xCurr, const VectorDynamic &xDelta) {
  size_t m = xDelta.rows();
  double minDelta = 1e9;
  // TODO: use an iterator instead
  for (uint i = 0; i < m; i++) {
    if (xDelta(i) < 0) {
      minDelta = std::min(minDelta, -1 * xCurr(i) / xDelta(i));
    }
  }
  return std::min(1.0, minDelta);
}

CentralVariable LPData::SolveLinearSystem() {
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S = centralVarCurr_.s.asDiagonal();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> X = centralVarCurr_.x.asDiagonal();
  VectorDynamic rb = A_ * centralVarCurr_.x - b_;
  VectorDynamic rc = A_.transpose() * centralVarCurr_.lambda + centralVarCurr_.s - c_;
  VectorDynamic rxs1 = X * centralVarCurr_.s;
  VectorDynamic sInv = centralVarCurr_.s;
  for (uint i = 0; i < sInv.rows(); i++)
    sInv(i) = 1 / sInv(i);
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_inv = sInv.asDiagonal();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> D2 = (S_inv * centralVarCurr_.x).asDiagonal();
  Eigen::SparseMatrix<double> AA = A_ * D2 * A_.transpose();
  // std::cout << "AA:\n" << Eigen::MatrixXd(AA) << std::endl;
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> AAFact(AA);
  // Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> AAFact;
  // AAFact.analyzePattern(AA);
  // AAFact.factorize(AA);

  // predictor
  CentralVariable centralDelta;
  centralDelta.lambda = AAFact.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs1));
  if (GlobalVariablesDAGOpt::debugMode == 1) {
    if ((eigen_is_nan(centralDelta.lambda)))
      CoutError("centralDelta.lambda becomes Nan during iterations!");
  }
  centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
  centralDelta.x = -1 * S_inv * rxs1 - X * (S_inv * centralDelta.s);

  // corrector
  double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
  double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
  double muAff = ((centralVarCurr_.x + centralDelta.x * alphaAffPrim).transpose() *
                  (centralVarCurr_.s + centralDelta.s * alphaAffDual))(0, 0) /
                 centralVarCurr_.x.rows();
  double sigma = std::pow(muAff / Duality(), 2);

  VectorDynamic rxs2 = -1 * (-1 * X * centralVarCurr_.s - centralDelta.x.asDiagonal() * centralDelta.s);
  VectorAdd(rxs2, Duality() * sigma * -1);

  centralDelta.lambda = AAFact.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs2));
  centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
  centralDelta.x = -1 * S_inv * rxs2 - X * (S_inv * centralDelta.s);

  return centralDelta;
}

void LPData::ApplyCentralDelta(const CentralVariable &centralDelta, double eta) {
  double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
  double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
  centralVarCurr_.x = centralVarCurr_.x + centralDelta.x * std::min(1.0, alphaAffPrim * eta);
  centralVarCurr_.lambda = centralVarCurr_.lambda + centralDelta.lambda * std::min(1.0, alphaAffDual * eta);
  centralVarCurr_.s = centralVarCurr_.s + centralDelta.s * std::min(1.0, alphaAffDual * eta);
}

VectorDynamic SolveLP(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const SpVec &c,
                      double precision) {
  LPData lpData(A, b, c);
  return SolveLP(lpData, precision);
}

VectorDynamic SolveLP(LPData &lpData, double precision) {
  if (lpData.centralVarCurr_.x.rows() == 0)
    lpData.centralVarCurr_ = lpData.GenerateInitialLP();
  int iterationCount = 0;
  while (lpData.Duality() > precision && iterationCount < 1000) {
    if (GlobalVariablesDAGOpt::debugMode == 1) {
      std::cout << "Current duality measure: " << lpData.Duality() << std::endl;
    }

    CentralVariable centralDelta = lpData.SolveLinearSystem();
    lpData.ApplyCentralDelta(centralDelta, 0.9);
    iterationCount++;
  }
  return lpData.centralVarCurr_.x.block(0, 0, lpData.n_, 1);
}

} // namespace LPOptimizer

namespace OrderOptDAG_SPACE {
using namespace LPOptimizer;

void RoundIPMResults(VectorDynamic &startTimeVector, double precision) {
  for (uint i = 0; i < startTimeVector.rows(); i++) {
    if (abs(startTimeVector(i) - round(startTimeVector(i))) < precision)
      startTimeVector(i) = round(startTimeVector(i));
  }
}

LPData GenerateRTDALPOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                         const std::vector<uint> processorJobVec, int processorNum,
                         bool lessJobOrderConstraints) {
  BeginTimer("GenerateRTDALPOrg");
  std::vector<AugmentedJacobianTriplet> jacobTripConstraints = GetDAGJacobianTripletOrg(
      dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum, lessJobOrderConstraints);

  jacobTripConstraints.reserve(jacobTripConstraints.size() + dagTasks.chains_.size());
  for (uint i = 0; i < dagTasks.chains_.size(); i++) {
    AugmentedJacobianTriplet augJacobRTDACurr = GetJacobianCauseEffectChainOrg(
        dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum, dagTasks.chains_[i], i);
    jacobTripConstraints.push_back(augJacobRTDACurr);
  }
  AugmentedSparseJacobian jacobAll = MergeAugJacobian(jacobTripConstraints);
  // VectorDynamic c = GenerateVectorDynamic(jacobAll.jacobian.cols());
  SpVec c(jacobAll.jacobian.cols());
  for (uint i = tasksInfo.length; i < c.rows(); i++)
    c.insert(i) = 1;
  LPData lpData(jacobAll.jacobian, jacobAll.rhs, c);
  EndTimer("GenerateRTDALPOrg");
  return lpData;
}

// TODO: add feasibility check
VectorDynamic OptRTDA_IPMOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                             SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum,
                             bool lessJobOrderConstraints) {

  LPData lpData = GenerateRTDALPOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum,
                                    lessJobOrderConstraints);
  if (GlobalVariablesDAGOpt::debugMode == 1)
    lpData.print();
  VectorDynamic startTimeVectorAfterOpt = SolveLP(lpData);
  RoundIPMResults(startTimeVectorAfterOpt);
  return startTimeVectorAfterOpt;
}

} // namespace OrderOptDAG_SPACE