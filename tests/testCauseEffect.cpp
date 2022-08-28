#include "unordered_map"
#include <CppUnitLite/TestHarness.h>

#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"

using namespace DAG_SPACE;
using namespace std;
// job data structure used in cause-effect chain analysis
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
template <>
struct std::hash<JobCEC>
{
    std::size_t operator()(const JobCEC &jobCEC) const
    {
        std::string str = std::to_string(jobCEC.taskId) + ", " + std::to_string(jobCEC.jobId);
        return std::hash<std::string>{}(str);
    }
};

// TODO: requires further test!
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

struct RTDA
{
    double reactionTime;
    double dataAge;
    RTDA() : reactionTime(-1), dataAge(-1) {}
    RTDA(double r, double d) : reactionTime(r), dataAge(d) {}
};

RTDA GetRTDAFromSingleJob(TaskSetInfoDerived &tasksInfo, std::vector<int> &causeEffectChain, Values &x)
{

    LLint hyperPeriod = tasksInfo.hyperPeriod;
    const TaskSet &tasks = tasksInfo.tasks;
    size_t totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period;
    RTDA res;
    std::vector<RTDA> resVec;
    resVec.reserve(totalStartJobs + 1);

    std::unordered_map<JobCEC, JobCEC> firstReactionMap;
    // Todo: Be careful! this termination condition is not consistent with Verucchi!
    for (size_t startInstanceIndex = 0; startInstanceIndex <= totalStartJobs + 1; startInstanceIndex++)
    {

        JobCEC firstJob = {causeEffectChain[0], startInstanceIndex};
        JobCEC lastJob = {-1, 0};
        bool findLastJob = false;
        for (uint j = 1; j < causeEffectChain.size(); j++)
        {
            double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
            size_t jobIndex = 0;
            while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
            {
                jobIndex++;
                if (jobIndex > 100)
                {
                    CoutError("didn't find a match!");
                }
            }
            firstJob = {causeEffectChain[j], jobIndex};
            if (jobIndex > 0)
            {
                lastJob = {causeEffectChain[j], jobIndex - 1};
                findLastJob = true;
            }
            else
            {
                CoutError("Should not gonna happen!");
            }
        }

        // TODO: Be careful about the last instance
        resVec.push_back(res);
        JobCEC jj(causeEffectChain[0], size_t(0));
        firstReactionMap[jj] = firstJob;
        resVec[startInstanceIndex].reactionTime = GetFinishTime(firstJob, x, tasksInfo) - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);

        // update data age
        if (startInstanceIndex > 0)
        {
            JobCEC lastReaction = firstJob;
            lastReaction.jobId--;
            resVec[startInstanceIndex - 1].dataAge = GetStartTime(lastReaction, x, tasksInfo) + tasks[lastReaction.taskId].executionTime -
                                                     GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
        }
    }
    return resVec[0];
}

void AddReactionTimeDataAgeFactor(NonlinearFactorGraph &graph,
                                  TaskSetInfoDerived &tasksInfo, DAG_SPACE::MAP_Prev &mapPrev)
{

    std::vector<int> causeEffectChain = {0, 1, 2};

    if (RtdaWeight == 0)
        return;

    std::vector<gtsam::Symbol> keysAll;
    keysAll.reserve(tasksInfo.length);
    for (uint i = 0; i < tasksInfo.tasks.size(); i++)
    {
        for (size_t j = 0; j < static_cast<size_t>(tasksInfo.hyperPeriod / tasksInfo.tasks[i].period); j++)
        {
            keysAll.push_back(GenerateKey(i, j));
        }
    }

    LambdaMultiKey f = [keysAll, tasksInfo, causeEffectChain](const Values &x)
    {
        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        size_t totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period;

        VectorDynamic res = GenerateVectorDynamic(totalStartJobs * 2);

        std::unordered_map<JobCEC, JobCEC> firstReactionMap;
        // Todo: Be careful! this termination condition is not consistent with Verucchi!
        for (size_t startInstanceIndex = 0; startInstanceIndex <= totalStartJobs + 1; startInstanceIndex++)
        {

            JobCEC firstJob = {causeEffectChain[0], 0};
            JobCEC lastJob = {-1, 0};
            bool findLastJob = false;
            for (uint j = 1; j < causeEffectChain.size(); j++)
            {
                double currentJobFT = GetFinishTime(firstJob, x, tasksInfo);
                size_t jobIndex = 0;
                while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) < currentJobFT)
                {
                    jobIndex++;
                    if (jobIndex > 100)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                firstJob = {causeEffectChain[j], jobIndex};
                if (jobIndex > 0)
                {
                    lastJob = {causeEffectChain[j], jobIndex - 1};
                    findLastJob = true;
                }
            }

            // TODO: Be careful, whether using < or <=
            if (startInstanceIndex < totalStartJobs)
            {
                JobCEC jj(causeEffectChain[0], size_t(0));
                firstReactionMap[jj] = firstJob;
                res(startInstanceIndex * 2, 0) = GetFinishTime(firstJob, x, tasksInfo) - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);
            }

            if (findLastJob && startInstanceIndex > 0)
            {
                auto p = firstReactionMap.find({causeEffectChain[0], startInstanceIndex - 1});
                if (p != firstReactionMap.end() && (p->second) != firstJob)
                {
                    res(startInstanceIndex * 2 - 1, 0) = GetStartTime(lastJob, x, tasksInfo) + tasks[lastJob.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
                }
            }
        }

        return res;
    };

    LLint errorDimensionRTDA = 2;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionRTDA, noiseModelSigma / RtdaWeight);

    graph.emplace_shared<MultiKeyFactor>(keysAll, f, tasksInfo.hyperPeriod / tasksInfo.tasks[causeEffectChain[0]].period * 2, model);
}

TEST(GetStartTime, v1)
{
    using namespace DAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v1.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

    EXPECT_LONGS_EQUAL(1, GetStartTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(11, GetFinishTime({0, 0}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(201, GetStartTime({0, 1}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(401, GetStartTime({0, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(403, GetStartTime({2, 2}, initialEstimateFG, tasksInfo));
    EXPECT_LONGS_EQUAL(405, GetStartTime({4, 2}, initialEstimateFG, tasksInfo));
    // EXPECT_LONGS_EQUAL(405, GetStartTime({5, 2}, initialEstimateFG, tasksInfo));
}

TEST(a, b)
{
    using namespace DAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v1.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    std::vector<int> causeEffectChain = {0, 1, 2};
    RTDA res = GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimateFG);
    EXPECT_LONGS_EQUAL(414, res.reactionTime);
    EXPECT_LONGS_EQUAL(414, res.dataAge);
}
// TEST(a, b)
// {
//     using namespace DAG_SPACE;
//     DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v1.csv", "orig"); // single-rate dag
//     TaskSet tasks = dagTasks.tasks;
//     TaskSetInfoDerived tasksInfo(tasks);

//     VectorDynamic initialEstimate = GenerateVectorDynamic(5);
//     initialEstimate << 1, 2, 3, 4, 5;
//     Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

//     NonlinearFactorGraph graph;
//     AddReactionTimeDataAgeFactor(graph, tasksInfo, dagTasks.mapPrev);
//     double errActual = graph.error(initialEstimateFG);
//     std::cout << "Actual error is " << errActual << std::endl;
//     // EXPECT_LONGS_EQUAL(414, errActual);
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
