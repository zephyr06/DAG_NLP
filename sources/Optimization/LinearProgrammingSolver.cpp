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

  AA_ = A_ * A_.transpose();
  BeginTimer("AnalyzePatternAA_");
  AASolver_.analyzePattern(AA_);
  AASolver_.factorize(AA_);
  EndTimer("AnalyzePatternAA_");

  centralVarCurr_ = GenerateInitialLP();
}
LPData::LPData(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const SpVec &c,
               const VectorDynamic &warmStartX)
    : LPData(A, b, c) {
  centralVarCurr_.x = warmStartX;
}
LPData::LPData(const LPData &lpData)
    : A_(lpData.A_), b_(lpData.b_), c_(lpData.c_), m_(lpData.m_), n_(lpData.n_),
      centralVarCurr_(lpData.centralVarCurr_), AA_(lpData.AA_) {
  // AASolver_ = Eigen::SimplicialLLT<Eigen::SparseMatrix<double>>();
  // Eigen::SparseMatrix<double> AA = A_ * A_.transpose();
  BeginTimer("AnalyzePatternAA_");
  AASolver_.analyzePattern(AA_);
  AASolver_.factorize(AA_);
  EndTimer("AnalyzePatternAA_");
}

LPData &LPData::operator=(const LPData &lpData) {
  A_ = lpData.A_;
  b_ = lpData.b_;
  c_ = lpData.c_;
  m_ = lpData.m_;
  n_ = lpData.n_;
  centralVarCurr_ = lpData.centralVarCurr_;
  AA_ = lpData.AA_;
  BeginTimer("AnalyzePatternAA_");
  AASolver_.analyzePattern(AA_);
  AASolver_.factorize(AA_);
  EndTimer("AnalyzePatternAA_");
  return *this;
}

CentralVariable LPData::GenerateInitialLP() {
  BeginTimer("GenerateInitialIPM");
  // Eigen::SparseMatrix<double> AA = A_ * A_.transpose();
  // Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> AAFact(AA_);
  // auto AAFact = solver.compute(AA);
  VectorDynamic xCurr_ = A_.transpose() * (AASolver_.solve(b_));
  VectorAdd(xCurr_, std::max(-1.5 * xCurr_.minCoeff(), 0.0));

  VectorDynamic lambdaCurr_ = AASolver_.solve(A_ * c_);
  VectorDynamic sCurr_ = c_ - A_.transpose() * lambdaCurr_;
  VectorAdd(sCurr_, std::max(-1.5 * sCurr_.minCoeff(), 0.0));
  double deltax = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / sCurr_.sum() / 2.0;
  double deltas = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / xCurr_.sum() / 2.0;
  VectorAdd(xCurr_, deltax);
  VectorAdd(sCurr_, deltas);
  EndTimer("GenerateInitialIPM");
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
  BeginTimer("SolveLinearSystem");
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S = centralVarCurr_.s.asDiagonal();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> X = centralVarCurr_.x.asDiagonal();
  VectorDynamic rb = A_ * centralVarCurr_.x - b_;
  VectorDynamic rc = A_.transpose() * centralVarCurr_.lambda + centralVarCurr_.s - c_;
  if (GlobalVariablesDAGOpt::debugMode == 1) {
    std::cout << "Current constraint violation measure: " << rb.norm() << ", " << rc.norm() << std::endl;
  }
  VectorDynamic rxs1 = X * centralVarCurr_.s;
  VectorDynamic sInv = centralVarCurr_.s;
  for (uint i = 0; i < sInv.rows(); i++)
    sInv(i) = 1 / sInv(i);
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S_inv = sInv.asDiagonal();
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> D2 = (S_inv * centralVarCurr_.x).asDiagonal();
  if (GlobalVariablesDAGOpt::debugMode == 1) {
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(AA);
    // double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    std::cout << Color::blue
              << "condition number of D2: " << D2.diagonal().maxCoeff() / D2.diagonal().minCoeff()
              << Color::def << std::endl;
    std::cout << Color::blue << "minimum element of D2: " << D2.diagonal().minCoeff() << Color::def
              << std::endl;
    if (D2.diagonal().minCoeff() < 1e-7) {
      VectorDynamic sth = S_inv * centralVarCurr_.x;
      VectorAdd(sth, GlobalVariablesDAGOpt::deltaOptimizer);
      D2 = sth.asDiagonal();
    }
  }

  BeginTimer("MatrixMul_AA");
  Eigen::SparseMatrix<double> AA = A_ * D2 * A_.transpose();

  // std::cout << "AA size: " << AA.size() << ", AA rows: " << AA.rows() << ", AA cols: " << AA.cols()
  //           << ", AA non-zeros: " << AA.nonZeros() << std::endl;
  EndTimer("MatrixMul_AA");
  // std::cout << "AA:\n" << Eigen::MatrixXd(AA) << std::endl;
  BeginTimer("MatrixFactorization:");
  // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> AAFact;
  // AAFact.compute(AA);
  AASolver_.factorize(AA);
  EndTimer("MatrixFactorization:");
  // Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> AAFact;
  // AAFact.analyzePattern(AA);
  // AAFact.factorize(AA);

  // predictor
  CentralVariable centralDelta;
  BeginTimer("Backward_substitution1");
  centralDelta.lambda = AASolver_.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs1));
  if (GlobalVariablesDAGOpt::debugMode == 1) {
    // Eigen::JacobiSVD<Eigen::MatrixXd> svd(AA);
    // double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    std::cout << Color::blue << "condition number of centralDelta.lambda: "
              << centralDelta.lambda.maxCoeff() / centralDelta.lambda.minCoeff() << Color::def << std::endl;
  }
  EndTimer("Backward_substitution1");
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
  BeginTimer("Backward_substitution2");
  centralDelta.lambda = AASolver_.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs2));
  EndTimer("Backward_substitution2");
  centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
  centralDelta.x = -1 * S_inv * rxs2 - X * (S_inv * centralDelta.s);
  EndTimer("SolveLinearSystem");
  return centralDelta;
}

void LPData::ApplyCentralDelta(const CentralVariable &centralDelta, double eta) {
  double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
  double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
  if (GlobalVariablesDAGOpt::debugMode == 1) {
    std::cout << "Current alpha_Aff_Prim and _dual: " << alphaAffPrim << ", " << alphaAffDual << std::endl;
  }
  centralVarCurr_.x = centralVarCurr_.x + centralDelta.x * std::min(1.0, alphaAffPrim * eta);
  centralVarCurr_.lambda = centralVarCurr_.lambda + centralDelta.lambda * std::min(1.0, alphaAffDual * eta);
  centralVarCurr_.s = centralVarCurr_.s + centralDelta.s * std::min(1.0, alphaAffDual * eta);
}

void LPData::ExamKKTCondition() const {
  std::cout << "Primal feasibility: " << (b_ - A_ * centralVarCurr_.x).maxCoeff() << ", "
            << (b_ - A_ * centralVarCurr_.x).minCoeff() << "\n";
  std::cout << "Dual feasibility: "
            << (VectorDynamic(c_) - A_.transpose() * centralVarCurr_.lambda - centralVarCurr_.s).maxCoeff()
            << ", "
            << (VectorDynamic(c_) - A_.transpose() * centralVarCurr_.lambda - centralVarCurr_.s).minCoeff()
            << "\n";
  std::cout << "Duality measure: " << Duality() << std::endl;
  std::cout << "Positve x: " << centralVarCurr_.x.minCoeff() << std::endl;
  std::cout << "Positve s: " << centralVarCurr_.s.minCoeff() << std::endl;
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
  double dualityPrev = lpData.Duality();
  while (dualityPrev > precision && iterationCount < 1000) {
    if (GlobalVariablesDAGOpt::debugMode == 1) {
      std::cout << "Current duality measure: " << lpData.Duality() << std::endl;
    }

    CentralVariable centralDelta = lpData.SolveLinearSystem();
    lpData.ApplyCentralDelta(centralDelta, 0.9);
    iterationCount++;
    double dualityCurr = lpData.Duality();
    // if (dualityCurr > dualityPrev * 100)
    //   CoutError("IPM diverged during iterations!");
    dualityPrev = dualityCurr;
  }
  std::cout << "IPM Iterations taken: " << iterationCount << std::endl;
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
  // if (GlobalVariablesDAGOpt::debugMode == 1)
  //   lpData.print();
  VectorDynamic startTimeVectorAfterOpt = SolveLP(lpData);
  if (GlobalVariablesDAGOpt::debugMode == 1)
    lpData.ExamKKTCondition();
  RoundIPMResults(startTimeVectorAfterOpt);
  return startTimeVectorAfterOpt;
}

} // namespace OrderOptDAG_SPACE