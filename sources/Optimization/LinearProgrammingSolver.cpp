#include "sources/Optimization/LinearProgrammingSolver.h"

namespace LPOptimizer
{

    // TODO: remove all the matrix inverse
    CentralVariable LPData::GenerateInitialLP()
    {
        MatrixDynamic AA = A_ * A_.transpose();
        VectorDynamic xCurr_ = A_.transpose() * (AA).inverse() * b_;
        VectorAdd(xCurr_, std::max(-1.5 * xCurr_.minCoeff(), 0.0));

        VectorDynamic lambdaCurr_ = AA.inverse() * A_ * c_;
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
        Eigen::MatrixXd S = centralVarCurr_.s.asDiagonal();
        Eigen::MatrixXd X = centralVarCurr_.x.asDiagonal();
        VectorDynamic rb = A_ * centralVarCurr_.x - b_;
        VectorDynamic rc = A_.transpose() * centralVarCurr_.lambda + centralVarCurr_.s - c_;

        // predictor
        VectorDynamic rxs1 = X * centralVarCurr_.s;
        Eigen::MatrixXd D2 = S.inverse() * X;
        CentralVariable centralDelta;
        centralDelta.lambda = (A_ * D2 * A_.transpose()).inverse() * (-1 * rb - A_ * X * S.inverse() * rc + A_ * (S.inverse() * rxs1));
        centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
        centralDelta.x = -1 * S.inverse() * rxs1 - X * (S.inverse() * centralDelta.s);

        // corrector
        double alphaAffPrim = GetAlphaAff(centralVarCurr_.x, centralDelta.x);
        double alphaAffDual = GetAlphaAff(centralVarCurr_.s, centralDelta.s);
        double muAff = ((centralVarCurr_.x + centralDelta.x * alphaAffPrim).transpose() * (centralVarCurr_.s + centralDelta.s * alphaAffDual))(0, 0) / centralVarCurr_.x.rows();
        double sigma = std::pow(muAff / Duality(), 3);

        VectorDynamic rxs2 = -1 * (-1 * X * centralVarCurr_.s - centralDelta.x.asDiagonal() * centralDelta.s);
        VectorAdd(rxs2, Duality() * sigma * -1);

        centralDelta.lambda = (A_ * D2 * A_.transpose()).inverse() * (-1 * rb - A_ * X * S.inverse() * rc + A_ * (S.inverse() * rxs2));
        centralDelta.s = -1 * rc - A_.transpose() * centralDelta.lambda;
        centralDelta.x = -1 * S.inverse() * rxs2 - X * (S.inverse() * centralDelta.s);

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

    VectorDynamic SolveLP(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c, double precision)
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
            lpData.ApplyCentralDelta(centralDelta, 0.7);
            iterationCount++;
        }
        return lpData.centralVarCurr_.x.block(0, 0, lpData.n_, 1);
    }

} // namespace LPOptimizer