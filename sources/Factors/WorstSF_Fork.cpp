#include "sources/Factors/WorstSF_Fork.h"

namespace OrderOptDAG_SPACE {

double GetMaxFinishTimeDiff(const std::vector<JobCEC> &last_reading_jobs,
                            const TaskSetInfoDerived &tasksInfo,
                            const VectorDynamic &startTimeVector) {
    double min_finish_time = 1e8;
    double max_finish_time = -1e8;
    for (auto job : last_reading_jobs) {
        double finish_time_curr =
            GetFinishTime(job, startTimeVector, tasksInfo);
        if (finish_time_curr < min_finish_time)
            min_finish_time = finish_time_curr;
        if (finish_time_curr > max_finish_time)
            max_finish_time = finish_time_curr;
    }
    return max_finish_time - min_finish_time;
}

std::vector<SF_JobFork> WorstSF_JobFork::FindWorstFork_DAG_Level(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum) {
    std::vector<SF_JobFork> res;
    res.reserve(dagTasks.sf_forks_.size() *
                3);  // a safe reservation that is probably not necessary
    for (const auto &sf_fork : dagTasks.sf_forks_) {
        std::vector<SF_JobFork> worst_sf_fork =
            FindWosrtFork(dagTasks, tasksInfo, jobOrder, startTimeVector,
                          processorNum, sf_fork);
        res.insert(res.end(), worst_sf_fork.begin(), worst_sf_fork.end());
    }
    return res;
}

std::vector<SF_JobFork> WorstSF_JobFork::FindWosrtFork(
    const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
    SFOrder &jobOrder, const VectorDynamic &startTimeVector, int processorNum,
    const SF_Fork &sf_fork) {
    double worst_case_sf_val = 0;
    std::vector<SF_JobFork> worst_sf;
    worst_sf.reserve(10);  // should be safe but not necessary
    int sink_id = sf_fork.sink;
    for (int sink_job_id = 0;
         sink_job_id < tasksInfo.hyper_period / tasksInfo.tasks[sink_id].period;
         sink_job_id++) {
        std::vector<JobCEC> last_reading_jobs = OptimizeSF::GetLastReadingJobs(
            JobCEC(sink_id, sink_job_id), sf_fork.source, jobOrder, tasksInfo);
        double source_time_dispa =
            GetMaxFinishTimeDiff(last_reading_jobs, tasksInfo, startTimeVector);
        if (source_time_dispa > worst_case_sf_val) {
            worst_case_sf_val = source_time_dispa;
            worst_sf.clear();
            worst_sf.push_back(
                SF_JobFork(JobCEC(sink_id, sink_job_id), last_reading_jobs));
        } else if (source_time_dispa == worst_case_sf_val) {
            worst_sf.push_back(
                SF_JobFork(JobCEC(sink_id, sink_job_id), last_reading_jobs));
        }
    }
    return worst_sf;
}
}  // namespace OrderOptDAG_SPACE