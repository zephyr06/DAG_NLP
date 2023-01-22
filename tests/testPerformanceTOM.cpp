
#include "sources/Utils/profilier.h"
#include "sources/batchOptimizeSFOrder.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace OrderOptDAG_SPACE;
TEST(average_performance, v1) {
  std::string dataSetFolder = GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/dagTasksPerfTest/";
  ClearResultFiles(dataSetFolder);
  std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod,
                                                                     OrderOptDAG_SPACE::TOM};
  std::vector<BatchResult> batchResVec =
      OrderOptDAG_SPACE::BatchOptimizeOrder<OptimizeSF::RTDAExperimentObj>(baselineMethods, dataSetFolder, 1);
  EXPECT_THAT(batchResVec[BaselineMethods::TOM].performance,
              testing::Le(18744.6 * 1.015)); // remove 1.015 if deciding giving up selectInitialFromPool
  // 18744.6 is achieved if simpleOrderScheduler finds feasible schedule for unschedulable job order
  EXPECT_DOUBLE_EQ(1.0, batchResVec[BaselineMethods::TOM].schedulableRatio);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}