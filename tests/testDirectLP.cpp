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
            0, 0, 1, 0,
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

class LPData
{
public:
    MatrixDynamic A_;
    VectorDynamic b_;
    VectorDynamic c_;
    size_t m_;
    size_t n_;

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

// Solve the following LP:
// min_x c^T x
// subject to Ax <= b
// This algorithm is based on primal-dual interior-point method, following the tutorial in Nocedal07Numerical_Optimization
// During the optimization process, this algorithm doesn't perform matrix permutation
VectorDynamic SolveLP(const MatrixDynamic &A, const VectorDynamic &b, const VectorDynamic &c)
{
    return b;
}

TEST_F(LPTest1, SolveLP)
{

    VectorDynamic xExpect(4);
    xExpect << 0, 10, 1, 0;
    VectorDynamic xActual = SolveLP(A, b, c);
    EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}