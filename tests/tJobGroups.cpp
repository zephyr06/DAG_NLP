#include <CppUnitLite/TestHarness.h>
#include "sources/Tools/testMy.h"
#include "sources/Optimization/Optimize.h"

using namespace DAG_SPACE;
TEST(FindJobIndexWithError, v1)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);

    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 46, 92.4;
    auto actual = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    EXPECT_LONGS_EQUAL(2, actual.size());
    // PrintKeyVector(actual[0]);
    // PrintKeyVector(actual[1]);
    EXPECT_LONGS_EQUAL(0, (actual[0][0]).taskId);
    EXPECT_LONGS_EQUAL(5, (actual[0][1]).taskId);
    EXPECT_LONGS_EQUAL(1, (actual[1][0]).taskId);
    EXPECT_LONGS_EQUAL(5, (actual[1][1]).taskId);
}

TEST(JobGroup, v1)
{
    std::vector<JobCEC> jobs1{JobCEC(3, 3), JobCEC(1, 1)};
    std::vector<JobCEC> jobs2{JobCEC(0, 0), JobCEC(1, 1), JobCEC(2, 2)};
    JobGroup g1(jobs2);
    JobGroup g2(jobs1);
    EXPECT(g1.existOverlap(jobs1));
    g1.insert(jobs1);
    EXPECT_LONGS_EQUAL(4, g1.size());
}

TEST(CreateJobGroups, v1)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);

    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 46, 92.4;
    auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);

    auto jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(2, jobGroups.size());
    EXPECT(jobGroups[0].exist(JobCEC(0, 1)));
    EXPECT(jobGroups[0].exist(JobCEC(5, 1)));
    EXPECT(jobGroups[1].exist(JobCEC(1, 0)));
    EXPECT(jobGroups[1].exist(JobCEC(5, 0)));
}

TEST(CreateJobGroups, v2)
{
    std::vector<std::vector<JobCEC>> jobPairsWithError;
    jobPairsWithError.push_back({JobCEC(0, 0), JobCEC(1, 1), JobCEC(2, 2)});
    jobPairsWithError.push_back({JobCEC(0, 0), JobCEC(3, 3), JobCEC(2, 2)});
    jobPairsWithError.push_back({JobCEC(5, 5), JobCEC(4, 4)});
    auto jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(2, jobGroups.size());
    EXPECT(jobGroups[0].exist(JobCEC(0, 0)));
    EXPECT(jobGroups[0].exist(JobCEC(1, 1)));
    EXPECT(jobGroups[0].exist(JobCEC(2, 2)));
    EXPECT(jobGroups[0].exist(JobCEC(3, 3)));
    EXPECT(jobGroups[1].exist(JobCEC(4, 4)));
    EXPECT(jobGroups[1].exist(JobCEC(5, 5)));
}

TEST(findRightJob, v1)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);
    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 46, 92.4;
    auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    auto jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(5, jobGroups[0].findRightJob(initialEstimate, tasksInfo).taskId);
    EXPECT_LONGS_EQUAL(1, jobGroups[1].findRightJob(initialEstimate, tasksInfo).taskId);

    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 50, 92.4;
    jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(2, jobGroups[0].findRightJob(initialEstimate, tasksInfo).taskId);
}

TEST(JobGroup, sort)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);
    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52.6, 76.5, 90.3, 27.9, 50, 92.4;
    auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    auto jobGroups = CreateJobGroups(jobPairsWithError);
    std::vector<JobCEC> actual = jobGroups[0].sort(initialEstimate, tasksInfo, "deadline");
    EXPECT_LONGS_EQUAL(0, actual[0].taskId);
    EXPECT_LONGS_EQUAL(5, actual[1].taskId);
    EXPECT_LONGS_EQUAL(2, actual[2].taskId);
}

TEST(JobGroup, SwitchRightJob)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
    dagTasks.tasks[2].deadline = 61;
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);
    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 46, 84.9, 5.56, 52, 76.5, 90.3, 27.9, 50, 92.4;

    auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    auto jobGroups = CreateJobGroups(jobPairsWithError);
    VectorDynamic actual = jobGroups[0].SwitchRightJob(initialEstimate, tasksInfo);
    EXPECT_LONGS_EQUAL(50, actual(4));
    EXPECT_LONGS_EQUAL(52, actual(8));

    // no deadline miss, leave the job to vanish gradient
    actual = jobGroups[1].SwitchRightJob(initialEstimate, tasksInfo);
    EXPECT_DOUBLES_EQUAL(5.56, actual(3), 0.1);
    EXPECT_DOUBLES_EQUAL(27.9, actual(7), 0.1);
}
TEST(JobGroup, MergeJobGroups)
{
    weightDDL_factor = 1;
    weightDAG_factor = 0;
    RtdaWeight = 0;
    DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n5_v70.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    NonlinearFactorGraph graph;
    BuildFactorGraph(dagTasks, graph, tasksInfo);
    VectorDynamic initialEstimate = GenerateInitial(dagTasks,
                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << -0.00705601,
        199.004,
        202,
        300,
        454,
        503,
        49.5819,
        304,
        47.5669,
        301,
        48.0087,
        302,
        0.0514781,
        204,
        455;

    auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
    std::vector<JobGroup> jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(1, jobGroups.size());
    EXPECT_LONGS_EQUAL(6, jobGroups[0].size());
}

TEST(MergeJobGroup, v2)
{
    std::vector<std::vector<JobCEC>> jobPairsWithError;
    jobPairsWithError.push_back({JobCEC(0, 0), JobCEC(1, 1), JobCEC(2, 2)});
    jobPairsWithError.push_back({JobCEC(9, 9), JobCEC(3, 3)});
    jobPairsWithError.push_back({JobCEC(5, 5), JobCEC(4, 4)});
    jobPairsWithError.push_back({JobCEC(8, 8), JobCEC(7, 7)});
    jobPairsWithError.push_back({JobCEC(10, 10), JobCEC(6, 6)});
    jobPairsWithError.push_back({JobCEC(1, 1), JobCEC(3, 3), JobCEC(4, 4), JobCEC(7, 7), JobCEC(6, 6)});
    auto jobGroups = CreateJobGroups(jobPairsWithError);
    EXPECT_LONGS_EQUAL(1, jobGroups.size());
    EXPECT_LONGS_EQUAL(11, (int)(jobGroups[0].size()));
}

// TEST(JobGroup, SwitchRightJob_v2)
// {
//     weightDDL_factor = 1;
//     weightDAG_factor = 0;
//     RtdaWeight = 0;
//     DAG_SPACE::DAG_Model dagTasks = DAG_SPACE::ReadDAG_Tasks(PROJECT_PATH + "TaskData/test_n6_v1.csv", "orig");
//     TaskSetInfoDerived tasksInfo(dagTasks.tasks);
//     NonlinearFactorGraph graph;
//     BuildFactorGraph(dagTasks, graph, tasksInfo);
//     VectorDynamic initialEstimate = GenerateInitial(dagTasks,
//                                                     tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
//     initialEstimate << 0, 45, 68, 5, 50, 60, 73, 27.9, 60.9, 75;
//     auto jobPairsWithError = FindJobIndexWithError(initialEstimate, tasksInfo, graph);
//     auto jobGroups = CreateJobGroups(jobPairsWithError);
//     VectorDynamic actual = jobGroups[0].SwitchRightJob(initialEstimate, tasksInfo);
//     EXPECT_LONGS_EQUAL(27.9, actual(4));
//     EXPECT_LONGS_EQUAL(5, actual(8));

//     actual = jobGroups[1].SwitchRightJob(initialEstimate, tasksInfo);
//     EXPECT_DOUBLES_EQUAL(60.9, actual(3), 0.1);
//     EXPECT_DOUBLES_EQUAL(60, actual(7), 0.1);
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}