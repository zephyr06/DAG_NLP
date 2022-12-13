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

    CentralVariable LPData::SolveLinearSystem()
    {
        Eigen::MatrixXd S = centralVarCurr_.s.asDiagonal();
        Eigen::MatrixXd X = centralVarCurr_.x.asDiagonal();
        VectorDynamic rb = A_ * centralVarCurr_.x - b_;
        VectorDynamic rc = A_.transpose() * centralVarCurr_.lambda + centralVarCurr_.s - c_;
        VectorDynamic rxs = X * centralVarCurr_.s;
        VectorAdd(rxs, Duality() * 0.5 * -1);
        Eigen::MatrixXd D2 = S.inverse() * X;
        CentralVariable deltaCentral;
        deltaCentral.lambda = (A_ * D2 * A_.transpose()).inverse() * (-1 * rb - A_ * X * S.inverse() * rc + A_ * (S.inverse() * rxs));
        deltaCentral.s = -1 * rc - A_.transpose() * deltaCentral.lambda;
        deltaCentral.x = -1 * S.inverse() * rxs - X * (S.inverse() * deltaCentral.s);
        return deltaCentral;
    }

} // namespace LPOptimizer