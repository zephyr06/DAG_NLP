#include "unordered_map"
#include <CppUnitLite/TestHarness.h>

#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"

using namespace DAG_SPACE;

// job data structure used in cause-effect chain analysis
struct JobCEC
{
    int taskId;
    size_t jobId;

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
struct hash<JobCEC>
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
    int jobNumInHyperPeriod = tasksInfo.hyperPeriod / tasksInfo.tasks[jobCEC.taskId].period;

    return x.at<VectorDynamic>(GenerateKey(jobCEC.taskId, jobCEC.jobId % jobNumInHyperPeriod))(0) + tasksInfo.tasks[jobCEC.taskId].executionTime + jobCEC.jobId / jobNumInHyperPeriod * tasksInfo.hyperPeriod;
}

void AddReactionTimeDataAgeFactor(NonlinearFactorGraph &graph,
                                  TaskSetInfoDerived &tasksInfo, DAG_SPACE::MAP_Prev &mapPrev)
{

    std::vector<int> causeEffectChain = {0, 1, 2};

    LLint errorDimensionMS = 1;
    if (RtdaWeight == 0)
        return;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma / RtdaWeight);

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

                double currentJobFT = GetStartTime(firstJob, x, tasksInfo);
                size_t jobIndex = 0;
                while (GetStartTime({causeEffectChain[j], jobIndex}, x, tasksInfo) > currentJobFT)
                {
                    jobIndex++;
                }
                firstJob = {causeEffectChain[j], jobIndex};
                if (jobIndex > 0)
                {
                    lastJob = {causeEffectChain[j], jobIndex - 1};
                    findLastJob = true;
                }
            }

            if (startInstanceIndex <= totalStartJobs)
            {
                firstReactionMap.insert({causeEffectChain[0], 0}, firstJob);
                res(startInstanceIndex * 2, 0) = GetStartTime(firstJob, x, tasksInfo) + tasks[firstJob.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex}, x, tasksInfo);
            }

            if (findLastJob && startInstanceIndex > 0)
            {
                auto p = firstReactionMap.find({causeEffectChain[0], startInstanceIndex - 1});
                if (p != firstReactionMap.end() && (p->second) != firstJob)
                {
                    res(startInstanceIndex * 2 - 1, 0) = GetStartTime(lastJob, x, tasksInfo) + tasks[lastJob.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
                }
            }

            // size_t startIndexOverall = IndexTran_Instance2Overall(causeEffectChain[0], startInstanceIndex, tasksInfo.sizeOfVariables);
            // double beginTime0 = x.at<VectorDynamic>(keysAll[startIndexOverall])(0); // start time of the $startInstanceIndex job \tau of the first task in chain

            // // start time of the current job T in a chain. we'll find T's dependent task in each for loop;
            // double beginTime = beginTime0;

            // // first reaction job to \tau during for-loop iterations
            // double firstReaction = beginTime0 + tasks[causeEffectChain[0]].executionTime;

            // double lastReaction = firstReaction; // last reaction job to \tau during for-loop iterations

            // double beginTimeNextInstance;
            // if (startInstanceIndex == totalStartJobs - 1)
            // {
            //     beginTimeNextInstance = beginTime + hyperPeriod;
            // }
            // else
            // {
            //     beginTimeNextInstance = x.at<VectorDynamic>(keysAll[IndexTran_Instance2Overall(causeEffectChain[0], startInstanceIndex + 1, tasksInfo.sizeOfVariables)])(0);
            // }
            // // go through each task in the chain
            // for (uint j = 1; j < causeEffectChain.size(); j++)
            // {
            //     int dependentTaskId = causeEffectChain[j];
            //     bool findFirstReact = false;
            //     bool findLastReact = false;
            //     size_t totalDependentInstance = static_cast<size_t>(hyperPeriod / tasks[dependentTaskId].period);
            //     for (size_t hyperIndex = 0; hyperIndex < causeEffectChain.size(); hyperIndex++)
            //     {
            //         for (size_t k = 0; k < totalDependentInstance; k++)
            //         {
            //             double startTime = x.at<VectorDynamic>(keysAll[IndexTran_Instance2Overall(dependentTaskId, k, tasksInfo.sizeOfVariables)])(0) + hyperIndex * hyperPeriod;

            //             if (!findFirstReact && startTime > firstReaction)
            //             {
            //                 findFirstReact = true;
            //                 firstReaction = startTime + tasks[dependentTaskId].executionTime;
            //             }
            //             if (findFirstReact && (!findLastReact) && startTime + tasks[dependentTaskId].executionTime <= beginTimeNextInstance)
            //             {
            //                 lastReaction = startTime + tasks[dependentTaskId].executionTime < beginTimeNextInstance;
            //             }
            //             if (findFirstReact && startTime + tasks[dependentTaskId].executionTime > beginTimeNextInstance)
            //             {
            //                 findLastReact = true;
            //                 break;
            //             }
            //         }
            //         // what if the cause-effect chain cannot be satisifed?
            //         if (!findFirstReact || !findLastReact)
            //         {
            //             CoutWarning("Didn't find match!");
            //         }
            //         if (findFirstReact && findLastReact)
            //         {
            //             break;
            //         }
            //     }
            // }
            // res(startInstanceIndex * 2, 0) = firstReaction - beginTime0;
            // res(startInstanceIndex * 2 + 1, 0) = lastReaction - beginTime0;
        }

        return res;
    };

    graph.emplace_shared<MultiKeyFactor>(keysAll, f, tasksInfo.hyperPeriod / tasksInfo.tasks[causeEffectChain[0]].period * 2, model);
}

TEST(a, b)
{
    using namespace DAG_SPACE;
    DAG_Model dagTasks = ReadDAG_Tasks("/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v1.csv", "orig"); // single-rate dag
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);

    // VectorDynamic initialEstimate = GenerateInitial(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, initialUser);
    VectorDynamic initialEstimate = GenerateVectorDynamic(5);
    initialEstimate << 1, 2, 3, 4, 5;
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);

    NonlinearFactorGraph graph;
    AddReactionTimeDataAgeFactor(graph, tasksInfo, dagTasks.mapPrev);
    double errActual = graph.error(initialEstimateFG);
    std::cout << "Actual error is " << errActual << std::endl;
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
