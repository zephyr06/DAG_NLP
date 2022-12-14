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
#include "sources/Optimization/LinearProgrammingSolver.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
using namespace LPOptimizer;
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
        As = A.sparseView();
        b = VectorDynamic(15);
        b << 10, 20, 20, 20, -0, -10, 0, -0, -1, -2, 0, 1, -1, 1, -3;
        c = VectorDynamic(4);
        c << 0, 1, 0, 1;
        // VectorDynamic xExpect(4);
        // xExpect << 0, 10, 1, 0;
        // for (int k = 0; k < As.outerSize(); ++k)
        // {
        //     for (Eigen::SparseMatrix<double>::InnerIterator it(As, k); it; ++it)
        //     {
        //         std::cout << "(" << it.row() << ","; // row index
        //         std::cout << it.col() << ")\t";      // col index (here it is equal to k)
        //     }
        // }
    };

    MatrixDynamic A;
    Eigen::SparseMatrix<double> As;
    VectorDynamic b;
    VectorDynamic c;
};

TEST_F(LPTest1, LPData_constructor)
{
    LPData lpData(As, b, c);
    EXPECT_EQ(A.sum() + 15, lpData.A_.sum());
    EXPECT_EQ(19, lpData.A_.cols());
    EXPECT_EQ(c.sum() + 0, lpData.c_.sum());
    EXPECT_TRUE(gtsam::assert_equal(b, lpData.b_));
}

TEST_F(LPTest1, GenerateInitialLP)
{
    LPData lpData(As, b, c);
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

TEST_F(LPTest1, SolveLP)
{

    TimerFunc _;
    VectorDynamic xExpect(4);
    xExpect << 0, 10, 1, 0;
    VectorDynamic xActual = SolveLP(As, b, c);
    EXPECT_TRUE(gtsam::assert_equal(xExpect, xActual, 1e-4));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}