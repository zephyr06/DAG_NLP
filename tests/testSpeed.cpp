
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "sources/Utils/profilier.h"
#include "sources/BatchOptimization/batchOptimizeSFOrder.h"

using namespace OrderOptDAG_SPACE;
TEST(speed, v1) {
    BeginTimer("main");
    int REPEAT = 10;
    OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(
        GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n5_v79.csv",
        "orig");

    ScheduleResult sth;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
    scheduleOption.LoadParametersYaml();
    scheduleOption.selectInitialFromPoolCandidate_ = 0;

    BeginTimer("SFOrderOpt");
    for (int i = 0; i < REPEAT; i++) {
        OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<
            SimpleOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::DataAgeObj>(
            tasks, scheduleOption);
    }
    EndTimer("SFOrderOpt");

    BeginTimer("Verucchi20");
    for (int i = 0; i < REPEAT; i++) {
        GetVerucchiRTDA(tasks, tasks.chains_,
                        GlobalVariablesDAGOpt::coreNumberAva,
                        GlobalVariablesDAGOpt::kVerucchiReactionCost,
                        GlobalVariablesDAGOpt::kVerucchiMaxReaction,
                        GlobalVariablesDAGOpt::kVerucchiDataAgeCost,
                        GlobalVariablesDAGOpt::kVerucchiMaxDataAge,
                        GlobalVariablesDAGOpt::kVerucchiCoreCost,
                        GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT);
    }
    EndTimer("Verucchi20");
    EndTimer("main");
    PrintTimer();
    EXPECT_GE(profilerMap["Verucchi20"].accum / profilerMap["SFOrderOpt"].accum,
              1.5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}