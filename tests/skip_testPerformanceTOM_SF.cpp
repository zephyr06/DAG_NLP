
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sources/Utils/profilier.h"
#include "sources/batchOptimizeSFOrder.h"

using namespace OrderOptDAG_SPACE;
TEST(average_performance, v1) {
    BeginTimer("main");
    std::string dataSetFolder =
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/dagTasksPerfTest/";
    ClearResultFiles(dataSetFolder);
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {
        OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM};
    std::vector<BatchResult> batchResVec =
        OrderOptDAG_SPACE::BatchOptimizeOrder<OptimizeSF::SensorFusionObj>(
            baselineMethods, dataSetFolder, 1);
    EXPECT_THAT(
        batchResVec[BASELINEMETHODS::TOM].performance,
        testing::Le(18744.6 * 1.015));  // remove 1.015 if deciding giving up
                                        // selectInitialFromPool
    // 18744.6 is achieved if simpleOrderScheduler finds feasible schedule for
    // unschedulable job order
    EXPECT_DOUBLE_EQ(1.0, batchResVec[BASELINEMETHODS::TOM].schedulableRatio);
    EndTimer("main");
    PrintTimer();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}