#ifndef VERUCCHI_RTDA_BRDIGE_H
#define VERUCCHI_RTDA_BRDIGE_H
// below are include files required by Verucchi
// #include "sources/Baseline/VerucchiScheduling.h"

// *************************
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Baseline/Verucchi20/DAG/DAG.h"
#include "sources/Baseline/Verucchi20/Evaluation/Scheduling.h"


TaskSet getTaskSet(DAG dag) {
    TaskSet tasks;
    auto nodes = dag.getOriginatingTaskset()->getNodes();
    for (auto node: nodes) {
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

// VectorDynamic getInitialEstimate(DAG dag) {
//     return getInitialEstimate(dag, 1);
// }

// defaut proposser number is 1
VectorDynamic getInitialEstimate(DAG dag, int nproc = 1) {
    int total_jobs = 0;
    std::vector<int> job_count;
    auto nodes = dag.getOriginatingTaskset()->getNodes();
    for (auto node: nodes) {
        job_count.push_back(node->nodes.size());
        total_jobs += node->nodes.size();
    }
    VectorDynamic initial_estimate = GenerateVectorDynamic(total_jobs);
    auto processorSchedule = 
    scheduling::getScheduleFromDAG(dag, nproc);
    
    for (size_t i = 0; i < processorSchedule.size(); i++) {
        float time = 0.0f;
        for (const auto &n : processorSchedule[i]) {
            if ( n->uniqueId > 1 && (int)n->uniqueId < (total_jobs + 2)) {
                initial_estimate(n->uniqueId-2,0) = time;
            }
            time += n->wcet;
        }
    }
    return initial_estimate;
}

#endif