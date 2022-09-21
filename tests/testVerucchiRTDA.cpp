
#include <CppUnitLite/TestHarness.h>

#include "sources/Baseline/VerucchiScheduling.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/VerucchiRTDABridge.h"
// *************************

TEST(VerucchiRTDA, v1)
{
    time_t tstart, tend;
    tstart = time(0);
    VariableTaskSet taskSet;

    auto task1 = taskSet.addTask(33, 5, "cameras");
    auto task2 = taskSet.addTask(99, 40, "detection");
    auto task3 = taskSet.addTask(99, 10, "lidar");
    auto task4 = taskSet.addTask(99, 8, "clustering");
    auto task5 = taskSet.addTask(99, 2, "fusion");
    auto task6 = taskSet.addTask(33, 5, "visualize");

    // task2->bcet = 27;
    // task4->bcet = 2;
    // task5->bcet = 1;

    taskSet.addDataEdge(task3, task4, {0});
    taskSet.addDataEdge(task1, task2, {0});
    taskSet.addDataEdge(task2, task5, {0});
    taskSet.addDataEdge(task4, task5, {0});
    taskSet.addDataEdge(task5, task6, {0});

    // taskSet.addPrecedenceEdge(task3, task4);
    // taskSet.addPrecedenceEdge(task1, task2);
    // taskSet.addPrecedenceEdge(task2, task5);
    // taskSet.addPrecedenceEdge(task4, task5);
    // taskSet.addPrecedenceEdge(task5, task6);

    taskSet.createBaselineTaskset();

    auto &allDags = taskSet.createDAGs();

    std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

    Evaluation eval;
    eval.addLatency({task1, task2, task5, task6}, LatencyCost(1, 15), LatencyConstraint(400, 400));
    // eval.addLatency({task3, task3, task4, task5, task6}, LatencyCost(1, 3), LatencyConstraint(400, 400));
    eval.addScheduling(SchedulingCost(20), SchedulingConstraint(4));

    const auto &bestDAG = eval.evaluate(allDags);

    // bestDAG.toTikz("fusion.tex");
    // bestDAG.getOriginatingTaskset()->toTikz("fusion_multi.tex");
    std::cout << bestDAG.getNodeInfo() << std::endl;
    // bestDAG.getLatencyInfo({1,1,2,3,4});
    scheduling::scheduleDAG(bestDAG, 4, "schedule_test.tex", true);
    tend = time(0);
    std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;

    TaskSet tasks = GetTaskSet(bestDAG);
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GetInitialEstimate(bestDAG, 4);
    std::cout << "Generated initial estimate from the bestDAG of Verucchi:\n";
    std::cout << initialEstimate.transpose() << std::endl;
    Values initialEstimateFG = DAG_SPACE::GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 4, 5};
    auto res = DAG_SPACE::GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    DAG_SPACE::RTDA resM = DAG_SPACE::GetMaxRTDA(res);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto task : causeEffectChain)
    {
        std::cout << task << ", ";
    }
    std::cout << "\nReaction time: " << resM.reactionTime << "\nData age: " << resM.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

TEST(VerucchiRTDA, v2)
{
    time_t tstart, tend;
    tstart = time(0);
    VariableTaskSet taskSet;

    auto task1 = taskSet.addTask(33, 5, "cameras");
    auto task2 = taskSet.addTask(99, 40, "detection");
    auto task3 = taskSet.addTask(99, 10, "lidar");
    auto task4 = taskSet.addTask(99, 8, "clustering");
    auto task5 = taskSet.addTask(99, 2, "fusion");
    auto task6 = taskSet.addTask(33, 5, "visualize");

    // task2->bcet = 27;
    // task4->bcet = 2;
    // task5->bcet = 1;

    taskSet.addDataEdge(task3, task4, {0});
    taskSet.addDataEdge(task1, task2, {0});
    taskSet.addDataEdge(task2, task5, {0});
    taskSet.addDataEdge(task4, task5, {0});
    taskSet.addDataEdge(task5, task6, {0});

    // taskSet.addPrecedenceEdge(task3, task4);
    // taskSet.addPrecedenceEdge(task1, task2);
    // taskSet.addPrecedenceEdge(task2, task5);
    // taskSet.addPrecedenceEdge(task4, task5);
    // taskSet.addPrecedenceEdge(task5, task6);

    taskSet.createBaselineTaskset();

    auto &allDags = taskSet.createDAGs();

    std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

    Evaluation eval;
    eval.addLatency({task1, task2, task5, task6}, LatencyCost(1, 15), LatencyConstraint(400, 400));
    // eval.addLatency({task3, task3, task4, task5, task6}, LatencyCost(1, 3), LatencyConstraint(400, 400));
    eval.addScheduling(SchedulingCost(20), SchedulingConstraint(4));

    const auto &bestDAG = eval.evaluateWithRTDA(allDags);

    // bestDAG.toTikz("fusion.tex");
    // bestDAG.getOriginatingTaskset()->toTikz("fusion_multi.tex");
    std::cout << bestDAG.getNodeInfo() << std::endl;
    // bestDAG.getLatencyInfo({1,1,2,3,4});
    scheduling::scheduleDAG(bestDAG, 4, "schedule_test.tex", true);
    tend = time(0);
    std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;

    TaskSet tasks = GetTaskSet(bestDAG);
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GetInitialEstimate(bestDAG, 4);
    std::cout << "Generated initial estimate from the bestDAG of Verucchi:\n";
    std::cout << initialEstimate.transpose() << std::endl;
    Values initialEstimateFG = DAG_SPACE::GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 4, 5};
    auto res = DAG_SPACE::GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    DAG_SPACE::RTDA resM = DAG_SPACE::GetMaxRTDA(res);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto task : causeEffectChain)
    {
        std::cout << task << ", ";
    }
    std::cout << "\nReaction time: " << resM.reactionTime << "\nData age: " << resM.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

TEST(VerucchiRTDA, single_case_v1)
{
    DAG_SPACE::DAG_Model tasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v1.csv", "orig");
    std::vector<int> causeEffectChain{0, 1, 2, 3};
    DAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, causeEffectChain, 1);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto task : causeEffectChain)
    {
        std::cout << task << ", ";
    }
    std::cout << "\nReaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

TEST(VerucchiRTDA, single_case_v2)
{
    DAG_SPACE::DAG_Model tasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v2.csv", "orig");
    std::vector<int> causeEffectChain{0, 1, 2, 3};

    DAG_SPACE::RTDA rtda = GetVerucchiRTDA(tasks, causeEffectChain, 1, 15.0, 400.0, 15.0, 400.0, 15.0);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto task : causeEffectChain)
    {
        std::cout << task << ", ";
    }
    std::cout << "\nReaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";

    rtda = GetVerucchiRTDA(tasks, causeEffectChain, 1, 15.0, 100.0, 15.0, 100.0, 15.0);
    std::cout << "<-------------RTDA results-------------->\nChain: ";
    for (auto task : causeEffectChain)
    {
        std::cout << task << ", ";
    }
    std::cout << "\nReaction time: " << rtda.reactionTime << "\nData age: " << rtda.dataAge << std::endl;
    std::cout << "<-------------End of RTDA results------->\n\n";
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
