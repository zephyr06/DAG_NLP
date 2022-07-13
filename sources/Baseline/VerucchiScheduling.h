
// below are include files required by Verucchi
#include <algorithm>
#include <iostream>
#include <set>

#include <eigen3/Eigen/Core>

#include "sources/Baseline/Verucchi20/DAG/MaxProduct.h"
#include "sources/Baseline/Verucchi20/Evaluation/Evaluation.h"
#include "sources/Baseline/Verucchi20/MultiRate/MultiRateTaskset.h"
#include "sources/Baseline/Verucchi20/VariableTaskSet/VariableTaskSet.h"
#include "sources/Baseline/Verucchi20/MultiRate/DummyNodes.h"
#include "sources/Baseline/Verucchi20/Evaluation/Scheduling.h"

#include "sources/TaskModel/DAG_Model.h"
// *************************

bool SchedulabilityAnalysisVerucchi(DAG_SPACE::DAG_Model &dagTasks, int processorsAvailable = coreNumberAva)
{

    VariableTaskSet taskSetVeru;

    // add tasks
    std::vector<std::shared_ptr<MultiNode>> tasksVecVeru;
    tasksVecVeru.reserve(dagTasks.tasks.size());
    for (size_t i = 0; i < dagTasks.tasks.size(); i++)
    {
        DAG_SPACE::Task &taskCurr = dagTasks.tasks[i];
        auto task = taskSetVeru.addTask(taskCurr.period, taskCurr.executionTime, std::to_string(taskCurr.id));
        tasksVecVeru.push_back(task);
    }

    // add edges
    DAG_SPACE::MAP_Prev &mapPrev = dagTasks.mapPrev;
    for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
    {
        TaskSet &tasksAfter = itr->second;
        for (uint i = 0; i < tasksAfter.size(); i++)
        {
            std::vector<uint> jitters(std::max(1, tasksAfter[i].period / dagTasks.tasks[itr->first].period));
            taskSetVeru.addDataEdge(tasksVecVeru[itr->first], tasksVecVeru[tasksAfter[i].id], jitters);
        }
    }

    // analyze
    taskSetVeru.createBaselineTaskset();
    auto &allDags = taskSetVeru.createDAGs();
    std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

    Evaluation eval;
    // eval.addLatency({task1, task1, task2, task5, task6}, LatencyCost(1, 15), LatencyConstraint(400, 400));
    // eval.addLatency({task3, task3, task4, task5, task6}, LatencyCost(1, 3), LatencyConstraint(400, 400));
    eval.addScheduling(SchedulingCost(20), SchedulingConstraint(processorsAvailable));
    const auto &bestDAG = eval.evaluate(allDags);

    bestDAG.toTikz("fusion.tex");
    bestDAG.getOriginatingTaskset()->toTikz("fusion_multi.tex");
    std::cout << bestDAG.getNodeInfo() << std::endl;
    // bestDAG.getLatencyInfo({1,1,2,3,4});
    bool whetherScheduleable = scheduling::scheduleDAG(bestDAG, processorsAvailable, "schedule_test.tex", true);

    return whetherScheduleable;
}