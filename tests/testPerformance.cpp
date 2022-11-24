
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "sources/batchOptimizeSFOrder.h"
#include "sources/Utils/profilier.h"

using namespace OrderOptDAG_SPACE;
TEST(average_performance, v1)
{
    std::string dataSetFolder = GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/dagTasksPerfTest/";
    ClearResultFiles(dataSetFolder);
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM_Fast};
    std::vector<BatchResult> batchResVec = OrderOptDAG_SPACE::BatchOptimizeOrder(baselineMethods, dataSetFolder);
    EXPECT_THAT(batchResVec[BaselineMethods::TOM_Fast].performance, testing::Le(18744.6));
    EXPECT_DOUBLE_EQ(1.0, batchResVec[BaselineMethods::TOM_Fast].schedulableRatio);
}
TEST(speed, v1)
{
    BeginTimer("main");
    int REPEAT = 10;
    OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv", "orig");

    BeginTimer("Verucchi20");
    for (int i = 0; i < REPEAT; i++)
    {
        OrderOptDAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, tasks.chains_, GlobalVariablesDAGOpt::coreNumberAva, GlobalVariablesDAGOpt::kVerucchiReactionCost, GlobalVariablesDAGOpt::kVerucchiMaxReaction,
                                                       GlobalVariablesDAGOpt::kVerucchiDataAgeCost, GlobalVariablesDAGOpt::kVerucchiMaxDataAge, GlobalVariablesDAGOpt::kVerucchiCoreCost, GlobalVariablesDAGOpt::kVerucchiTimeLimit);
    }
    EndTimer("Verucchi20");

    ScheduleResult sth;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
    scheduleOption.LoadParametersYaml();

    BeginTimer("SFOrderOpt");
    for (int i = 0; i < REPEAT; i++)
    {
        sth = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(tasks, scheduleOption);
    }
    EndTimer("SFOrderOpt");
    EndTimer("main");
    PrintTimer();
    EXPECT_GE(profilerMap["Verucchi20"].accum / profilerMap["SFOrderOpt"].accum, 5.0);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}