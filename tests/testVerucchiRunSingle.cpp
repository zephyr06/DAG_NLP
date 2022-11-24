
#include <CppUnitLite/TestHarness.h>
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
// *************************
using namespace GlobalVariablesDAGOpt;
TEST(VerucchiRTDA, multichains_debug_use_only)
{
    OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    OrderOptDAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, tasks.chains_, GlobalVariablesDAGOpt::coreNumberAva, kVerucchiReactionCost, kVerucchiMaxReaction,
                                                   kVerucchiDataAgeCost, kVerucchiMaxDataAge, kVerucchiCoreCost, GlobalVariablesDAGOpt::kVerucchiTimeLimit);
    std::cout << "<-------------RTDA results-------------->\n";
    tasks.printChains();
    std::cout << "Reaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
