#pragma once

#include "sources/Factors/ObjectiveRTDA.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
class RTSS21ICObj : ObjectiveFunctionBase {
   public:
    static const std::string type_trait;
    static double Evaluate(const DAG_Model &dagTasks,
                           const TaskSetInfoDerived &tasksInfo,
                           const VectorDynamic &startTimeVector,
                           const ScheduleOptions scheduleOptions);

    static double TrueObj(const DAG_Model &dagTasks,
                          const TaskSetInfoDerived &tasksInfo,
                          const VectorDynamic &startTimeVector,
                          const ScheduleOptions scheduleOptions);

    static double EvaluateRTDA(const DAG_Model &dagTasks,
                               const TaskSetInfoDerived &tasksInfo,
                               const VectorDynamic &startTimeVector,
                               const ScheduleOptions scheduleOptions);

    static double EvaluateSF(const DAG_Model &dagTasks,
                             const TaskSetInfoDerived &tasksInfo,
                             const VectorDynamic &startTimeVector,
                             const ScheduleOptions scheduleOptions);
};

std::vector<JobCEC> GetLastReadingJobs(JobCEC sink_job,
                                       const std::vector<int> &source_ids,
                                       SFOrder &jobOrder,
                                       const TaskSetInfoDerived &tasksInfo);

}  // namespace OptimizeSF
}  // namespace OrderOptDAG_SPACE