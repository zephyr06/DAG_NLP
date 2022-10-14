#ifndef VERUCCHI_RTDA_BRDIGE_H_
#define VERUCCHI_RTDA_BRDIGE_H_
// below are include files required by Verucchi
// #include "sources/Baseline/VerucchiScheduling.h"

// *************************
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/Verucchi20/DAG/DAG.h"
#include "sources/Baseline/Verucchi20/Evaluation/Scheduling.h"
#include "sources/Baseline/Verucchi20/VariableTaskSet/VariableTaskSet.h"
#include "sources/Optimization/OptimizeOrder.h"

TaskSet GetTaskSet(DAG dag)
{
    TaskSet tasks;
    auto nodes = dag.getOriginatingTaskset()->getNodes();
    for (auto node : nodes)
    {
        RegularTaskSystem::Task task;

        task.offset = 0;
        task.period = node->period;
        task.overhead = 0;
        task.executionTime = node->wcet;
        task.deadline = node->deadline;
        task.id = node->id;

        tasks.push_back(task);
    }
    return tasks;
}

// defaut proposser number is 1
VectorDynamic GetInitialEstimate(const DAG &dag, int nproc = 1)
{
    if (dag.getPeriod() == 0)
    {
        VectorDynamic empty;
        return empty;
    }
    int total_jobs = 0;
    std::vector<int> job_count;
    auto nodes = dag.getOriginatingTaskset()->getNodes();
    for (auto node : nodes)
    {
        job_count.push_back(node->nodes.size());
        total_jobs += node->nodes.size();
    }
    VectorDynamic initial_estimate = GenerateVectorDynamic(total_jobs);
    auto processorSchedule = scheduling::getScheduleFromDAG(dag, nproc);

    for (size_t i = 0; i < processorSchedule.size(); i++)
    {
        float time = 0.0f;
        for (const auto &n : processorSchedule[i])
        {
            if (n->uniqueId > 1 && (int)n->uniqueId < (total_jobs + 2))
            {
                initial_estimate(n->uniqueId - 2, 0) = time;
            }
            time += n->wcet;
        }
    }
    return initial_estimate;
}

void GenerateVariableTaskSetFromDAGModel(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    VariableTaskSet &taskSetVeru,
    std::vector<std::shared_ptr<MultiNode>> &tasksVecVeru)
{
    tasksVecVeru.reserve(dagTasks.tasks.size());
    for (size_t i = 0; i < dagTasks.tasks.size(); i++)
    {
        OrderOptDAG_SPACE::Task &taskCurr = dagTasks.tasks[i];
        auto task = taskSetVeru.addTask(taskCurr.period, taskCurr.executionTime, std::to_string(taskCurr.id));
        tasksVecVeru.push_back(task);
    }
    // add edges
    OrderOptDAG_SPACE::MAP_Prev &mapPrev = dagTasks.mapPrev;
    for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
    {
        TaskSet &tasksAfter = itr->second;
        for (uint i = 0; i < tasksAfter.size(); i++)
        {
            // fix jitter to {0}
            taskSetVeru.addDataEdge(tasksVecVeru[tasksAfter[i].id], tasksVecVeru[itr->first], {0, 1, 2});
        }
    }
}

DAG FindTheBestDag(
    const std::vector<DAG> &allDags,
    const std::vector<std::shared_ptr<MultiNode>> &tasksVecVeru,
    const std::vector<std::vector<int>> &causeEffectChains,
    int processorsAvailable = coreNumberAva,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50)
{
    Evaluation eval;
    for (auto causeEffectChain : causeEffectChains)
    {
        std::vector<std::shared_ptr<MultiNode>> chain;
        for (auto task_id : causeEffectChain)
        {
            chain.push_back(tasksVecVeru[task_id]);
        }
        eval.addLatency(chain, LatencyCost(ageCost, reactCost), LatencyConstraint(maxAge, maxReact));
    }
    eval.addScheduling(SchedulingCost(coreCost), SchedulingConstraint(processorsAvailable));

    const auto &bestDAG = eval.evaluateWithRTDA(allDags);
    if (bestDAG.getNumNodes() <= 2)
    {
        if (debugMode)
        {
            std::cout << "Failed to find a valid dag.\n";
        }
        return DAG(0);
    }
    bool verucchiSuccess = scheduling::scheduleDAG(bestDAG, processorsAvailable, "schedule_test.tex", true);
    if (!verucchiSuccess)
    {
        if (debugMode)
        {
            std::cout << "Best dag is not schedulable.\n";
        }
        return DAG(0);
    }
    if (debugMode)
    {
        std::cout << bestDAG.getNodeInfo() << std::endl;
    }

    return DAG(bestDAG);
}

OrderOptDAG_SPACE::RTDA GetRTDAFromBestDag(
    const DAG &bestDAG,
    const std::vector<std::vector<int>> &causeEffectChains,
    int processorsAvailable = coreNumberAva)
{
    if (bestDAG.getPeriod() == 0)
    {
        return OrderOptDAG_SPACE::RTDA();
    }

    TaskSet tasks = GetTaskSet(bestDAG);
    TaskSetInfoDerived tasksInfo(tasks);
    VectorDynamic initialEstimate = GetInitialEstimate(bestDAG, processorsAvailable);
    if (debugMode)
    {
        std::cout << "Generated initial estimate from the bestDAG of Verucchi:\n";
        OrderOptDAG_SPACE::PrintSchedule(tasksInfo, initialEstimate);
    }
    Values initialEstimateFG = OrderOptDAG_SPACE::GenerateInitialFG(initialEstimate, tasksInfo);
    OrderOptDAG_SPACE::RTDA resM(0, 0);
    for (auto causeEffectChain : causeEffectChains)
    {
        auto res = OrderOptDAG_SPACE::GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
        auto max_of_current_chain = OrderOptDAG_SPACE::GetMaxRTDA(res);
        resM.dataAge += max_of_current_chain.dataAge;
        resM.reactionTime += max_of_current_chain.reactionTime;
    }
    return resM;
}

// get verucchi's RTDA on multiple chains
OrderOptDAG_SPACE::RTDA GetVerucchiRTDA(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    std::vector<std::vector<int>> causeEffectChains,
    int processorsAvailable = coreNumberAva,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50,
    int64_t time_limit = INT64_MAX)
{

    VariableTaskSet taskSetVeru;
    std::vector<std::shared_ptr<MultiNode>> tasksVecVeru;
    GenerateVariableTaskSetFromDAGModel(dagTasks, taskSetVeru, tasksVecVeru);
    // generate all dags
    taskSetVeru.createBaselineTaskset();
    // add time limits when create dags
    auto &allDags = taskSetVeru.createDAGsWithTimeLimit(time_limit);
    if (debugMode)
    {
        std::cout << allDags.size() << " total valid DAGs were created" << std::endl;
    }

    DAG bestDAG = FindTheBestDag(allDags, tasksVecVeru, causeEffectChains, processorsAvailable,
                                 reactCost, maxReact, ageCost, maxAge, coreCost);

    return GetRTDAFromBestDag(bestDAG, causeEffectChains, processorsAvailable);
}

OrderOptDAG_SPACE::ScheduleResult ScheduleVerucchiRTDA(
    OrderOptDAG_SPACE::DAG_Model &dagTasks,
    std::vector<std::vector<int>> causeEffectChains,
    int processorsAvailable,
    float reactCost = 15.0,
    float maxReact = 400.0,
    float ageCost = 15.0,
    float maxAge = 400.0,
    unsigned coreCost = 50,
    int64_t time_limit = kVerucchiTimeLimit)
{
    OrderOptDAG_SPACE::ScheduleResult res;
    VariableTaskSet taskSetVeru;
    std::vector<std::shared_ptr<MultiNode>> tasksVecVeru;
    GenerateVariableTaskSetFromDAGModel(dagTasks, taskSetVeru, tasksVecVeru);
    taskSetVeru.createBaselineTaskset();
    // generate all dags with a time limit
    auto &allDags = taskSetVeru.createDAGsWithTimeLimit(time_limit);
    if (debugMode)
    {
        std::cout << allDags.size() << " total valid DAGs were created" << std::endl;
    }

    DAG bestDAG = FindTheBestDag(allDags, tasksVecVeru, causeEffectChains, processorsAvailable,
                                 reactCost, maxReact, ageCost, maxAge, coreCost);

    res.rtda_ = GetRTDAFromBestDag(bestDAG, causeEffectChains, processorsAvailable);
    res.startTimeVector_ = GetInitialEstimate(bestDAG, processorsAvailable);
    if (res.rtda_.reactionTime < 0 || res.rtda_.dataAge < 0)
    {
        res.schedulable_ = false;
    }
    else
    {
        res.schedulable_ = true;
    }
    res.obj_ = ObjRTDA(res.rtda_);
    return res;
}

#endif // VERUCCHI_RTDA_BRDIGE_H_