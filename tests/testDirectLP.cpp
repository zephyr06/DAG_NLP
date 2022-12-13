#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Jacobi>
#include "gtsam/base/Testable.h" // assert_equal

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/JacobianAnalyze.h"
#include "sources/Utils/IncrementQR.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class LPTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        A = MatrixDynamic(15, 4);
        A << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1,
            -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, -1,
            1, 0, -1, 0,
            0, -1, 1, 0,
            1, 0, 0, -1,
            -1, 0, 0, 1,
            1, 0, -1, 0,
            0, 0, 1, -1,
            0, -1, 0, 1;
        b = VectorDynamic(15);
        b << 10, 20, 20, 20, -0, -10, 0, -0, -1, -2, 0, 1, -1, 1, -3;
        c = VectorDynamic(4);
        c << 0, 1, 0, 1;
        // VectorDynamic xExpect(4);
        // xExpect << 0, 10, 1, 0;
    };

    MatrixDynamic A;
    VectorDynamic b;
    VectorDynamic c;
};

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
    VectorDynamic x;
    VectorDynamic s;
    VectorDynamic lambda;
};
class LPData
{
public:
    MatrixDynamic A_;
    VectorDynamic b_;
    VectorDynamic c_;
    size_t m_;
    size_t n_;
    CentralVariable centralVarCurr_;

    LPData() {}
    LPData(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c) : b_(b), m_(A.rows()), n_(A.cols())
    {
        if (m_ > n_)
        {
            VectorDynamic onesForDiag = Eigen::MatrixXd::Ones(m_, 1);
            MatrixDynamic onesDiag = (onesForDiag).asDiagonal();
            // A_.resize(m_, n_ + m_);
            A_ = Eigen::MatrixXd(m_, m_ + n_);
            A_ << A, onesDiag;
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
    CentralVariable GenerateInitialLP()
    {
        MatrixDynamic AA = A_ * A_.transpose();
        VectorDynamic xCurr_ = A_.transpose() * (AA).inverse() * b_;
        VectorAdd(xCurr_, std::max(-1.5 * xCurr_.minCoeff(), 0.0));

        VectorDynamic lambdaCurr_ = AA.inverse() * A_ * c_;
        VectorDynamic sCurr_ = c_ - A_.transpose() * lambdaCurr_;
        VectorAdd(sCurr_, std::max(-1.5 * sCurr_.minCoeff(), 0.0));
        double deltax = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / sCurr_.sum() / 2.0;
        double deltas = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / sCurr_.sum() / 2.0;
        VectorAdd(xCurr_, deltax);
        VectorAdd(sCurr_, deltas);

        return CentralVariable{xCurr_, sCurr_, lambdaCurr_};
    }

    // void
};

TEST_F(LPTest1, LPData_constructor)
{
    LPData lpData(A, b, c);
    EXPECT_EQ(A.sum() + 15, lpData.A_.sum());
    EXPECT_EQ(19, lpData.A_.cols());
    EXPECT_EQ(c.sum() + 0, lpData.c_.sum());
    EXPECT_TRUE(gtsam::assert_equal(b, lpData.b_));
}

TEST_F(LPTest1, GenerateInitialLP)
{
    LPData lpData(A, b, c);
    VectorDynamic x0Expect = GenerateVectorDynamic(19);
    x0Expect << 6.5639,
        11.4870,
        8.5062,
        8.0062,
        7.3844,
        12.4613,
        15.4421,
        15.9421,
        6.5639,
        1.4870,
        8.5062,
        8.0062,
        2.9165,
        2.9549,
        3.4165,
        1.5318,
        2.9165,
        2.4742,
        2.4549;

    EXPECT_TRUE(gtsam::assert_equal(x0Expect, lpData.centralVarCurr_.x, 1e-4));
}

// Solve the following LP:
// min_x c^T x
// subject to Ax <= b
// This algorithm is based on primal-dual interior-point method, following the tutorial in Nocedal07Numerical_Optimization
// During the optimization process, this algorithm doesn't perform matrix permutation
// TODO: avoid more data copy/paste
VectorDynamic SolveLP(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c)
{
    LPData lpDataExpand(A, b, c);
    return b;
}

// TEST_F(LPTest1, SolveLP)
// {

//     VectorDynamic xExpect(4);
//     xExpect << 0, 10, 1, 0;
//     VectorDynamic xActual = SolveLP(A, b, c);
//     EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual));
// }

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}