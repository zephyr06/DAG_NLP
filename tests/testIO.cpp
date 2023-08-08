#include "gtsam/base/Testable.h" // assert_equal
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "gmock/gmock.h" // Brings in gMock.
#include <Eigen/Core>
#include <Eigen/Jacobi>
#include <Eigen/SparseCore>
#include <gtest/gtest.h>

#include "sources/Utils/Parameters.h"

using namespace GlobalVariablesDAGOpt;

TEST(IO, yaml)
{

  std::cout << GlobalVariablesDAGOpt::PROJECT_PATH << "\n";
  // << GlobalVariablesDAGOpt::coreNumberAva << "\n";
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}