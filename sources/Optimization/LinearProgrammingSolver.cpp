#include "sources/Optimization/LinearProgrammingSolver.h"
// TODO: if there is time, consider using band matrix from eigen
namespace LPOptimizer
{
    LPData::LPData(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c) : b_(b), m_(A.rows()), n_(A.cols())
    {
        if (m_ > n_)
        {
            // A_.resize(m_, n_ + m_);
            A_ = Eigen::SparseMatrix<double>(m_, m_ + n_);
            A_.reserve(A.nonZeros() + m_);

            for (uint c = 0; c < A.cols(); ++c)
            {
                A_.startVec(c); // Important: Must be called once for each column before inserting!
                for (Eigen::SparseMatrix<double>::InnerIterator itL(A, c); itL; ++itL)
                    A_.insertBack(itL.row(), c) = itL.value();
            }
            A_.finalize();
            for (uint i = 0; i < m_; i++)
                A_.insert(i, i + n_) = 1;

            c_ = Eigen::MatrixXd(m_ + n_, 1);
            VectorDynamic zeros = GenerateVectorDynamic(m_);
            c_ << c,
                zeros;
        }
        else
        {
            CoutError("A's dimension is wrong in LPData!");
        }
        centralVarCurr_ = GenerateInitialLP();
    }

    CentralVariable LPData::GenerateInitialLP()
    {
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

    double GetAlphaAff(const VectorDynamic &xCurr, const VectorDynamic &xDelta)
    {
        size_t m = xDelta.rows();
        double minDelta = 1e9;
        // TODO: use an iterator instead
        for (uint i = 0; i < m; i++)
        {
            if (xDelta(i) < 0)
            {
                minDelta = std::min(minDelta, -1 * xCurr(i) / xDelta(i));
            }
        }
        return std::min(1.0, minDelta);
    }

    CentralVariable LPData::SolveLinearSystem()
    {
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
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> AAFact(AA);

        // predictor
        CentralVariable centralDelta;
        centralDelta.lambda = AAFact.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs1));
        centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
        centralDelta.x = -1 * S_inv * rxs1 - X * (S_inv * centralDelta.s);

        // corrector
        double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
        double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
        double muAff = ((centralVarCurr_.x + centralDelta.x * alphaAffPrim).transpose() * (centralVarCurr_.s + centralDelta.s * alphaAffDual))(0, 0) / centralVarCurr_.x.rows();
        double sigma = std::pow(muAff / Duality(), 2);

        VectorDynamic rxs2 = -1 * (-1 * X * centralVarCurr_.s - centralDelta.x.asDiagonal() * centralDelta.s);
        VectorAdd(rxs2, Duality() * sigma * -1);

        centralDelta.lambda = AAFact.solve(-1 * rb - A_ * X * S_inv * rc + A_ * (S_inv * rxs2));
        centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
        centralDelta.x = -1 * S_inv * rxs2 - X * (S_inv * centralDelta.s);

        return centralDelta;
    }

    void LPData::ApplyCentralDelta(const CentralVariable &centralDelta, double eta)
    {
        double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
        double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
        centralVarCurr_.x = centralVarCurr_.x + centralDelta.x * std::min(1.0, alphaAffPrim * eta);
        centralVarCurr_.lambda = centralVarCurr_.lambda + centralDelta.lambda * std::min(1.0, alphaAffDual * eta);
        centralVarCurr_.s = centralVarCurr_.s + centralDelta.s * std::min(1.0, alphaAffDual * eta);
    }

    VectorDynamic SolveLP(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c, double precision)
    {
        LPData lpData(A, b, c);
        int iterationCount = 0;
        while (lpData.Duality() > precision && iterationCount < 1000)
        {
            if (GlobalVariablesDAGOpt::debugMode == 1)
            {
                std::cout << "Current duality measure: " << lpData.Duality() << std::endl;
            }

            CentralVariable centralDelta = lpData.SolveLinearSystem();
            lpData.ApplyCentralDelta(centralDelta, 0.9);
            iterationCount++;
        }
        return lpData.centralVarCurr_.x.block(0, 0, lpData.n_, 1);
    }

} // namespace LPOptimizer