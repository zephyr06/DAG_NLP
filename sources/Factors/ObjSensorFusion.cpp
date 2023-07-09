#include "sources/Factors/ObjSensorFusion.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
// TODO: add "SensorFusionObj"
const std::string RTSS21ICObj::type_trait("RTSS21ICObj");
const std::string SensorFusionObj::type_trait("SensorFusionObj");

double RTSS21ICObj::EvaluateRTDA(const DAG_Model &dagTasks,
                                 const TaskSetInfoDerived &tasksInfo,
                                 const VectorDynamic &startTimeVector,
                                 const ScheduleOptions scheduleOptions) {
    std::vector<std::vector<RTDA>> rtdaVec =
        GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
    std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
    double overallRTDA = GetOverallRtda(rtdaVec);

    double resFromRTDA = overallRTDA * scheduleOptions.weightInMpRTDA_;
    for (uint i = 0; i < dagTasks.chains_.size(); i++) {
        resFromRTDA +=
            Barrier(scheduleOptions.freshTol_ - maxRtda[i].reactionTime) *
                scheduleOptions.weightPunish_ +
            Barrier(scheduleOptions.freshTol_ - maxRtda[i].dataAge) *
                scheduleOptions.weightPunish_;
    }
    return resFromRTDA;
}

double RTSS21ICObj::EvaluateSF(const DAG_Model &dagTasks,
                               const TaskSetInfoDerived &tasksInfo,
                               const VectorDynamic &startTimeVector,
                               const ScheduleOptions scheduleOptions) {
    VectorDynamic sfVec =
        ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);

    double sfOverall = sfVec.sum();
    double resSF = sfOverall * scheduleOptions.weightInMpSf_;
    if (sfVec.rows() > 0)
        resSF +=
            Barrier(scheduleOptions.sensorFusionTolerance_ - sfVec.maxCoeff()) *
            scheduleOptions.weightPunish_;
    return resSF;
}

double RTSS21ICObj::Evaluate(const DAG_Model &dagTasks,
                             const TaskSetInfoDerived &tasksInfo,
                             const VectorDynamic &startTimeVector,
                             const ScheduleOptions scheduleOptions) {
    return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) +
           EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
}

double RTSS21ICObj::TrueObj(const DAG_Model &dagTasks,
                            const TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector,
                            const ScheduleOptions scheduleOptions) {
    return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) +
           EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
}

std::vector<JobCEC> GetLastReadingJobs(JobCEC sink_job,
                                       const std::vector<int> &source_ids,
                                       SFOrder &jobOrder,
                                       const TaskSetInfoDerived &tasksInfo) {
    std::vector<JobCEC> last_reading_jobs;
    last_reading_jobs.reserve(source_ids.size());
    for (int source_id : source_ids) {
        last_reading_jobs.push_back(
            FindLastReadingJob(sink_job, source_id, jobOrder, tasksInfo));
    }
    return last_reading_jobs;
}

double SensorFusionObj::TrueObj(const DAG_Model &dagTasks,
                                const TaskSetInfoDerived &tasksInfo,
                                const VectorDynamic &startTimeVector,
                                const ScheduleOptions scheduleOptions) {
    // VectorDynamic sfVec =
    //     ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);
    // return sfVec.maxCoeff();
    SFOrder jobOrder(tasksInfo, startTimeVector);
    double total_sf_error = 0;
    for (auto &sf_fork : dagTasks.sf_forks_) {
        int sink_id = sf_fork.sink;
        auto source_ids = sf_fork.source;
        double max_sf_diff = 0;
        for (int sink_job_id = 0;
             sink_job_id <
             tasksInfo.hyper_period / tasksInfo.tasks[sink_id].period;
             sink_job_id++) {
            JobCEC sink_job(sink_id, sink_job_id);
            std::vector<JobCEC> last_reading_jobs =
                GetLastReadingJobs(sink_job, source_ids, jobOrder, tasksInfo);
            double sf_obj_curr_fork =
                FindEarlyAndLateJob(SF_JobFork(sink_job, last_reading_jobs),
                                    startTimeVector, tasksInfo)
                    .GetFinishTimeDiff(startTimeVector, tasksInfo);
            max_sf_diff = std::max(max_sf_diff, sf_obj_curr_fork);
        }
        total_sf_error += max_sf_diff;
    }
    return total_sf_error;
}

double SensorFusionObj::Evaluate(const DAG_Model &dagTasks,
                                 const TaskSetInfoDerived &tasksInfo,
                                 const VectorDynamic &startTimeVector,
                                 const ScheduleOptions scheduleOptions) {
    SFOrder jobOrder(tasksInfo, startTimeVector);
    double total_sf_error = 0;
    for (auto &sf_fork : dagTasks.sf_forks_) {
        int sink_id = sf_fork.sink;
        auto source_ids = sf_fork.source;
        double max_sf_diff = 0;
        for (int sink_job_id = 0;
             sink_job_id <
             tasksInfo.hyper_period / tasksInfo.tasks[sink_id].period;
             sink_job_id++) {
            JobCEC sink_job(sink_id, sink_job_id);
            std::vector<JobCEC> last_reading_jobs =
                GetLastReadingJobs(sink_job, source_ids, jobOrder, tasksInfo);
            double sf_obj_curr_fork =
                FindEarlyAndLateJob(SF_JobFork(sink_job, last_reading_jobs),
                                    startTimeVector, tasksInfo)
                    .GetFinishTimeDiff(startTimeVector, tasksInfo);
            total_sf_error += sf_obj_curr_fork;
        }
    }
    return total_sf_error;
}
}  // namespace OptimizeSF
}  // namespace OrderOptDAG_SPACE