#pragma once
#include "sources/Factors/ObjSensorFusion.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE {

double GetMaxFinishTimeDiff(const std::vector<JobCEC> &last_reading_jobs,
                            const TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector);

class WorstSF_JobFork {
   public:
    WorstSF_JobFork() {}
    WorstSF_JobFork(const DAG_Model &dagTasks,
                    const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                    const VectorDynamic &startTimeVector, int processorNum)
        : worst_fork_(FindWorstFork_DAG_Level(dagTasks, tasksInfo, jobOrder,
                                              startTimeVector, processorNum)) {}

    std::vector<SF_JobFork> FindWorstFork_DAG_Level(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum);

    std::vector<SF_JobFork> FindWosrtFork(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum, const SF_Fork &sf_fork, double tolerance = 0.1);

    size_t size() const { return worst_fork_.size(); }
    SF_JobFork operator[](size_t index) const { return worst_fork_[index]; }

    // data members
    // there could be multiple forks with the same source time disparity
    std::vector<SF_JobFork> worst_fork_;
};

}  // namespace OrderOptDAG_SPACE