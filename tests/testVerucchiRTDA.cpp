
#include <CppUnitLite/TestHarness.h>

#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
// *************************

TEST(VerucchiRTDA, single_case_v1)
{
    OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig");
    std::vector<std::vector<int>> causeEffectChains{{0, 1, 2, 3}};
    OrderOptDAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, causeEffectChains, 1);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto chain : causeEffectChains)
    {
        for (auto task : chain)
        {
            std::cout << task << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "\nReaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

// TEST(VerucchiRTDA, single_case_v2)
// {
//     OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
//     std::vector<std::vector<int>> causeEffectChains{{0, 1, 4, 5}};

//     OrderOptDAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, causeEffectChains, 1, 15.0, 4000.0, 15.0, 4000.0, 15.0);
//     std::cout << "<-------------RTDA results-------------->\n";
//     for (auto chain : causeEffectChains)
//     {
//         for (auto task : chain)
//         {
//             std::cout << task << ", ";
//         }
//         std::cout << std::endl;
//     }
//     std::cout << "Reaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
//     std::cout << "<-------------End of RTDA results------->\n\n";
// }

TEST(VerucchiRTDA, single_case_multiple_chains)
{
    OrderOptDAG_SPACE::DAG_Model tasks = OrderOptDAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");

    OrderOptDAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, tasks.chains_, 1, 15.0, 4000.0, 15.0, 4000.0, 15.0);
    std::cout << "<-------------RTDA results-------------->\n";
    tasks.printChains();
    std::cout << "Reaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";

    if (!whether_shuffle_CE_chain)
    {
        if (tasks.chains_.size() == 1)
        {
            EXPECT_DOUBLES_EQUAL(99, rtda.reactionTime, 1e-3);
            EXPECT_DOUBLES_EQUAL(109, rtda.dataAge, 1e-3);
        }
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
