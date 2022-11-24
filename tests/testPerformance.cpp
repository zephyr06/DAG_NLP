
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "sources/batchOptimizeSFOrder.h"

using namespace OrderOptDAG_SPACE;
TEST(average_performance, v1)
{
    std::string dataSetFolder = PROJECT_PATH + "TaskData/dagTasksPerfTest/";
    ClearResultFiles(dataSetFolder);
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM_Fast};
    std::vector<BatchResult> batchResVec = OrderOptDAG_SPACE::BatchOptimizeOrder(baselineMethods, dataSetFolder);
    EXPECT_THAT(batchResVec[BaselineMethods::TOM_Fast].performance, testing::Le(18744.6));
    // EXPECT_EQ(1.0, batchResVec[BaselineMethods::TOM_Fast].schedulableRatio);
    EXPECT_DOUBLE_EQ(1.0, batchResVec[BaselineMethods::TOM_Fast].schedulableRatio);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}