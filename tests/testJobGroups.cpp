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

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}