
#include "sources/Optimization/OptimizeSFOrder_TOM_Threshold.h"
using namespace GlobalVariablesDAGOpt;
void testThreshold()
{
    BeginTimer("main");
    using namespace OrderOptDAG_SPACE;
    int chainNum = GlobalVariablesDAGOpt::NumCauseEffectChain;
    DAG_Model dagTasks =
        ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" +
                          testDataSetName + ".csv",
                      "orig", chainNum);

    ScheduleResult sth;
    OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
    scheduleOption.LoadParametersYaml();
    // LPOrderScheduler
    sth = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel_Threshold<LPOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::ReactionTimeObj>(dagTasks, scheduleOption);
    PrintResultAnalyzation(sth, dagTasks);
    std::cout << "Schedulable? " << sth.schedulable_ << std::endl;
    EndTimer("main");
    PrintTimer();
}

int main()
{
    testThreshold();
}