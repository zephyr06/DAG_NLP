#include "unordered_map"

#include "gtsam/base/Value.h"

#include "sources/TaskModel/RegularTasks.h"

namespace DAG_SPACE
{
    struct JobCEC
    {
        int taskId;
        size_t jobId;
        JobCEC() : taskId(-1), jobId(0) {}
        JobCEC(int taskId, size_t jobId) : taskId(taskId), jobId(jobId) {}

        bool operator==(const JobCEC &other) const
        {
            return taskId == other.taskId && jobId == other.jobId;
        }
        bool operator!=(const JobCEC &other) const
        {
            return !(*this == other);
        }
    };

    double GetStartTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo.N)
        {
            CoutError("GetStartTime receives invalid jobCEC!");
        }
        int jobNumInHyperPeriod = tasksInfo.hyperPeriod / tasksInfo.tasks[jobCEC.taskId].period;

        double res = x.at<VectorDynamic>(GenerateKey(jobCEC.taskId, jobCEC.jobId % jobNumInHyperPeriod))(0) + jobCEC.jobId / jobNumInHyperPeriod * tasksInfo.hyperPeriod;
        return res;
    }

    inline double GetFinishTime(JobCEC jobCEC, const Values &x, const TaskSetInfoDerived &tasksInfo)
    {
        return GetStartTime(jobCEC, x, tasksInfo) + tasksInfo.tasks[jobCEC.taskId].executionTime;
    }
}

template <>
struct std::hash<DAG_SPACE::JobCEC>
{
    std::size_t operator()(const DAG_SPACE::JobCEC &jobCEC) const
    {
        std::string str = std::to_string(jobCEC.taskId) + ", " + std::to_string(jobCEC.jobId);
        return std::hash<std::string>{}(str);
    }
};