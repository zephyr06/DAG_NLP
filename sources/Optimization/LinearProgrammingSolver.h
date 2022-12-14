#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/Jacobi>

#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/testMy.h"     // CoutError
#include "sources/Utils/Parameters.h" // DebugMode

namespace LPOptimizer
{

    typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
    // in-place addition, add c to each x_i
    inline void VectorAdd(VectorDynamic &x, double c)
    {
        x = x + Eigen::MatrixXd::Ones(x.rows(), 1) * c;
    }

    struct CentralVariable
    {
        CentralVariable() {}
        CentralVariable(VectorDynamic x,
                        VectorDynamic s,
                        VectorDynamic lambda) : x(x), s(s), lambda(lambda) {}

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

        inline CentralVariable &operator+(const CentralVariable &a)
        {
            x = x + a.x;
            s = s + a.s;
            lambda = lambda + a.lambda;
            return *this;
        }

        inline CentralVariable &operator-(const CentralVariable &a)
        {
            x = x - a.x;
            s = s - a.s;
            lambda = lambda - a.lambda;
            return *this;
        }

        VectorDynamic x;
        VectorDynamic s;
        VectorDynamic lambda;
    };

    class LPData
    {
    public:
        Eigen::SparseMatrix<double> A_;
        VectorDynamic b_;
        VectorDynamic c_;
        size_t m_;
        size_t n_;
        CentralVariable centralVarCurr_;

        LPData() {}
        // LPData(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c) : b_(b), m_(A.rows()), n_(A.cols())
        // {
        //     if (m_ > n_)
        //     {
        //         VectorDynamic onesForDiag = Eigen::MatrixXd::Ones(m_, 1);
        //         MatrixDynamic onesDiag = (onesForDiag).asDiagonal();
        //         // A_.resize(m_, n_ + m_);
        //         Eigen::MatrixXd A_t(m_, m_ + n_);
        //         A_t << A, onesDiag;
        //         A_ = A_t.sparseView();

        //         c_ = Eigen::MatrixXd(m_ + n_, 1);
        //         VectorDynamic zeros = GenerateVectorDynamic(m_);
        //         c_ << c,
        //             zeros;
        //     }
        //     else
        //     {
        //         CoutError("A's dimension is wrong in LPData!");
        //     }
        //     centralVarCurr_ = GenerateInitialLP();
        // }

        LPData(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c) : b_(b), m_(A.rows()), n_(A.cols())
        {
            if (m_ > n_)
            {
                VectorDynamic onesForDiag = Eigen::MatrixXd::Ones(m_, 1);
                Eigen::DiagonalMatrix<double, Eigen::Dynamic> onesDiag = (onesForDiag).asDiagonal();
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

        // TODO: remove all the matrix inverse
        CentralVariable GenerateInitialLP();

        CentralVariable SolveLinearSystem();

        void ApplyCentralDelta(const CentralVariable &centralDelta, double eta = 0.7);

        inline double Duality() const
        {
            return (centralVarCurr_.x.transpose() * centralVarCurr_.s / centralVarCurr_.s.rows())(0, 0);
        }
    };

    // Solve the following LP:
    // min_x c^T x
    // subject to Ax <= b
    // This algorithm is based on primal-dual interior-point method, following the tutorial in Nocedal07Numerical_Optimization
    // During the optimization process, this algorithm doesn't perform matrix permutation
    // TODO: avoid more data copy/paste
    VectorDynamic SolveLP(const Eigen::SparseMatrix<double> &A, const VectorDynamic &b, const VectorDynamic &c, double precision = 1e-5);
} // namespace LPOptimizer