
#include <CppUnitLite/TestHarness.h>

#include "sources/Baseline/VerucchiScheduling.h"
// *************************

TEST(Verucchi, v1)
{
    time_t tstart, tend;
    tstart = time(0);
    VariableTaskSet taskSet;

    std::string name = "cameras";
    auto task1 = taskSet.addTask(33, 5, name);
    auto task2 = taskSet.addTask(99, 40, "detection");
    auto task3 = taskSet.addTask(99, 10, "lidar");
    auto task4 = taskSet.addTask(99, 8, "clustering");
    auto task5 = taskSet.addTask(99, 2, "fusion");
    auto task6 = taskSet.addTask(33, 5, "visualize");

    // task2->bcet = 27;
    // task4->bcet = 2;
    // task5->bcet = 1;

    taskSet.addDataEdge(task3, task4, {0});
    taskSet.addDataEdge(task1, task2, {0, 1, 2});
    taskSet.addDataEdge(task2, task5, {0});
    taskSet.addDataEdge(task4, task5, {0});
    taskSet.addDataEdge(task5, task6, {0, 1, 2});

    // taskSet.addPrecedenceEdge(task3, task4);
    // taskSet.addPrecedenceEdge(task1, task2);
    // taskSet.addPrecedenceEdge(task2, task5);
    // taskSet.addPrecedenceEdge(task4, task5);
    // taskSet.addPrecedenceEdge(task5, task6);

    taskSet.createBaselineTaskset();

    auto &allDags = taskSet.createDAGs();

    std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

    Evaluation eval;
    eval.addLatency({task1, task1, task2, task5, task6}, LatencyCost(1, 15), LatencyConstraint(400, 400));
    eval.addLatency({task3, task3, task4, task5, task6}, LatencyCost(1, 3), LatencyConstraint(400, 400));
    eval.addScheduling(SchedulingCost(20), SchedulingConstraint(4));

    const auto &bestDAG = eval.evaluate(allDags);

    bestDAG.toTikz("fusion.tex");
    bestDAG.getOriginatingTaskset()->toTikz("fusion_multi.tex");
    std::cout << bestDAG.getNodeInfo() << std::endl;
    // bestDAG.getLatencyInfo({1,1,2,3,4});
    scheduling::scheduleDAG(bestDAG, coreNumberAva, "schedule_test.tex", true);

    tend = time(0);
    std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;
}

TEST(VerucchiIO, v2)
{
    DAG_SPACE::DAG_Model tasks = DAG_SPACE::ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n6_v1.csv", "orig");
    EXPECT(SchedulabilityAnalysisVerucchi(tasks, 4));
}

TEST(Verucchi, analyzeCase)
{
    std::cout << std::endl
              << std::endl
              << std::endl;
    DAG_SPACE::DAG_Model tasks = DAG_SPACE::ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/" + testDataSetName + ".csv", "orig");
    bool schedulable = SchedulabilityAnalysisVerucchi(tasks);
    std::cout << "Schedulability analysis by Verucchi: " << schedulable << std::endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
