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
        double deltas = 0.5 * (xCurr_.transpose() * sCurr_)(0, 0) / xCurr_.sum() / 2.0;
        VectorAdd(xCurr_, deltax);
        VectorAdd(sCurr_, deltas);

        return CentralVariable{xCurr_, sCurr_, lambdaCurr_};
    }

    CentralVariable SolveLinearSystem()
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

    inline void ApplyCentralDelta(const CentralVariable &centralDelta)
    {
        centralVarCurr_ = centralVarCurr_ + centralDelta;
    }

    inline double Duality() const
    {
        return (centralVarCurr_.x.transpose() * centralVarCurr_.s / centralVarCurr_.s.rows())(0, 0);
    }
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
    VectorDynamic s0Expect = GenerateVectorDynamic(19);
    s0Expect << 0.5843,
        0.7574,
        0.5891,
        0.7141,
        0.4048,
        0.2317,
        0.4000,
        0.2750,
        0.5843,
        0.7574,
        0.5891,
        0.7141,
        0.4994,
        0.6628,
        0.6244,
        0.3647,
        0.4994,
        0.6195,
        0.5378;
    VectorDynamic lambda0Expect = GenerateVectorDynamic(15);
    lambda0Expect << 0.0897,
        0.2628,
        0.0946,
        0.2196,
        -0.0897,
        -0.2628,
        -0.0946,
        -0.2196,
        -0.0048,
        -0.1683,
        -0.1298,
        0.1298,
        -0.0048,
        -0.1250,
        -0.0433;

    EXPECT_TRUE(gtsam::assert_equal(x0Expect, lpData.centralVarCurr_.x, 1e-4));
    EXPECT_TRUE(gtsam::assert_equal(s0Expect, lpData.centralVarCurr_.s, 1e-4));
    EXPECT_TRUE(gtsam::assert_equal(lambda0Expect, lpData.centralVarCurr_.lambda, 1e-4));
}

TEST_F(LPTest1, SolveLinearSystem)
{
    LPData lpData(A, b, c);
    VectorDynamic deltaLambdaExpect = GenerateVectorDynamic(15);
    deltaLambdaExpect << -0.4062,
        -0.5019,
        -0.2759,
        -0.3572,
        -0.3924,
        -0.1712,
        -0.1973,
        -0.1986,
        -0.7254,
        -0.2528,
        -0.7032,
        -1.7754,
        -0.7254,
        -1.3167,
        -0.2845;
    VectorDynamic deltasExpect = GenerateVectorDynamic(19);
    deltasExpect << -0.1021,
        -0.7012,
        -0.2972,
        -0.2959,
        -0.0883,
        0.0073,
        -0.2186,
        -0.1373,
        -0.1021,
        -0.3234,
        -0.2972,
        -0.2959,
        0.2308,
        -0.2417,
        0.2087,
        1.2809,
        0.2308,
        0.8222,
        -0.2100;
    VectorDynamic deltaxExpect = GenerateVectorDynamic(19);
    deltaxExpect << -2.4527,
        1.4345,
        -1.2752,
        -2.2634,
        -1.4956,
        -5.3828,
        -2.6731,
        -1.6850,
        -2.4527,
        1.4345,
        -1.2752,
        -2.2634,
        -0.7967,
        0.7355,
        -1.7848,
        -2.1635,
        -0.7967,
        -2.9623,
        1.7237;

    CentralVariable centralDelta = lpData.SolveLinearSystem();

    EXPECT_TRUE(gtsam::assert_equal(deltaLambdaExpect, centralDelta.lambda, 1e-4));
    EXPECT_TRUE(gtsam::assert_equal(deltasExpect, centralDelta.s, 1e-4));
    EXPECT_TRUE(gtsam::assert_equal(deltaxExpect, centralDelta.x, 1e-4));
}

// Solve the following LP:
// min_x c^T x
// subject to Ax <= b
// This algorithm is based on primal-dual interior-point method, following the tutorial in Nocedal07Numerical_Optimization
// During the optimization process, this algorithm doesn't perform matrix permutation
// TODO: avoid more data copy/paste
VectorDynamic SolveLP(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c, double precision = 1e-5)
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
        lpData.ApplyCentralDelta(centralDelta);
        iterationCount++;
    }
    return lpData.centralVarCurr_.x.block(0, 0, lpData.n_, 1);
    ;
}

TEST_F(LPTest1, SolveLP)
{

    VectorDynamic xExpect(4);
    xExpect << 0, 10, 1, 0;
    VectorDynamic xActual = SolveLP(A, b, c);
    EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual, 1e-4));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}