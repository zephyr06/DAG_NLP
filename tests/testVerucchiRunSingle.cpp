
#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/profilier.h"
#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
// *************************

TEST(VerucchiRTDA, multichains_debug_use_only)
{
    BeginTimer("main");
    DAG_SPACE::DAG_Model tasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig");

    DAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, tasks.chains_, 1, kVerucchiReactionCost, kVerucchiMaxReaction,
                                           kVerucchiDataAgeCost, kVerucchiMaxDataAge, kVerucchiCoreCost, kVerucchiTimeLimit);
    std::cout << "<-------------RTDA results-------------->\n";
    tasks.printChains();
    std::cout << "Reaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";

    EndTimer("main");
    PrintTimer();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
