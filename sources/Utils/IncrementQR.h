#pragma once
#include "sources/Optimization/JacobianAnalyze.h"

namespace OrderOptDAG_SPACE
{

    struct QR
    {
        MatrixDynamic Q;
        MatrixDynamic R;
        void print()
        {
            std::cout << "Q" << std::endl;
            std::cout << Q << std::endl;
            std::cout << "R" << std::endl;
            std::cout << R << std::endl;
        }
    };
    QR GetEigenQR(const MatrixDynamic &A)
    {
        Eigen::MatrixXd Q(A.rows(), A.rows());
        Eigen::MatrixXd R(A.rows(), A.cols());

        /////////////////////////////////HouseholderQR////////////////////////
        Eigen::MatrixXd thinQ(A.rows(), A.cols()), q(A.rows(), A.rows());

        Eigen::HouseholderQR<Eigen::MatrixXd> householderQR(A);
        q = householderQR.householderQ();
        thinQ.setIdentity();
        Q = householderQR.householderQ() * thinQ;

        std::cout << "HouseholderQR" << std::endl;

        R = householderQR.matrixQR().template triangularView<Eigen::Upper>();
        return QR{Q, R};
    }
} // namespace OrderOptDAG_SPACE