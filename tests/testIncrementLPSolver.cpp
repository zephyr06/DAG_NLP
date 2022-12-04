#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "gtsam/base/Testable.h" // assert_equal

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
class DAGScheduleOptimizerTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/test_n3_v18.csv", "orig", 1);
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        chain1 = {0, 2};
        dagTasks.chains_[0] = chain1;
        timeLimits = 1;
        scheduleOptions.processorNum_ = 2;
        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;

        initial = GenerateVectorDynamic(4);
        initial << 0, 10, 15, 12;
        jobOrder = SFOrder(tasksInfo, initial);
        jobOrder.print();
        VectorDynamic initial2 = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
    };

    double timeLimits = 1;
    DAG_Model dagTasks;
    TaskSet tasks;
    TaskSetInfoDerived tasksInfo;
    std::vector<int> chain1;
    ScheduleOptions scheduleOptions;
    std::vector<uint> processorJobVec;
    SFOrder jobOrder;
    VectorDynamic initial;
};

struct AugmentedJacobian
{
    MatrixDynamic jacobian;
    VectorDynamic rhs;
    void print()
    {
        std::cout << "Jacobian: " << jacobian << std::endl;
        std::cout << "RHS: " << rhs << std::endl;
    }
};
AugmentedJacobian GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
{
    MatrixDynamic jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);

    VectorDynamic rhs = GenerateVectorDynamic(tasksInfo.length);
    int count = 0;
    for (uint i = 0; i < dagTasks.tasks.size(); i++)
    {
        for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
        {
            jacobian(count, count) = 1;
            rhs(count++, 0) = GetDeadline(JobCEC{(int)i, j}, tasksInfo);
        }
    }
    return AugmentedJacobian{jacobian, rhs};
}

AugmentedJacobian GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
{
    MatrixDynamic jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);

    VectorDynamic rhs = GenerateVectorDynamic(tasksInfo.length);
    int count = 0;
    for (uint i = 0; i < dagTasks.tasks.size(); i++)
    {
        for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
        {
            jacobian(count, count) = -1;
            rhs(count++, 0) = GetActivationTime(JobCEC{(int)i, j}, tasksInfo) * -1;
        }
    }
    return AugmentedJacobian{jacobian, rhs};
}

// TODO: what if s and f are not continuous
AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder)
{
    int m = tasksInfo.length - 1;
    MatrixDynamic jacobian = GenerateMatrixDynamic(m, tasksInfo.length);
    VectorDynamic rhs = GenerateVectorDynamic(m);
    int count = 0;
    const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
    for (uint i = 1; i < instanceOrder.size(); i++)
    {
        auto instCurr = instanceOrder[i];
        auto instPrev = instanceOrder[i - 1];
        if (instPrev.job == instCurr.job)
            continue;
        int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo);
        int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo);

        if (count > 2)
            CoutError("Please provide implementation for discontinuous 's' and 'f' in GetJacobianJobOrder");
        jacobian(count, globalIdCurr) = -1;
        jacobian(count, globalIdPrev) = 1;
        if (instPrev.type == 's')
        {
            if (instCurr.type == 's')
            {
                rhs(count) = 0;
            }
            else // instCurr.type == 'f'
            {
                rhs(count) = GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        else // instPrev.type == 'f'
        {
            if (instCurr.type == 's')
            {
                rhs(count) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
            }
            else // type == 'f'
            {
                rhs(count) = -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        count++;
    }

    return AugmentedJacobian{jacobian, rhs};
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDDL)
{
    auto augJaco = GetJacobianDDL(dagTasks, tasksInfo);
    EXPECT_EQ(tasksInfo.length, augJaco.jacobian.sum());
    EXPECT_EQ(1, augJaco.jacobian(0, 0));
    VectorDynamic rhsExpect = GenerateVectorDynamic(4);
    rhsExpect << 10, 20, 20, 20;
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    augJaco.print();
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianActivationTime)
{
    auto augJaco = GetJacobianActivationTime(dagTasks, tasksInfo);
    EXPECT_EQ(-1 * tasksInfo.length, augJaco.jacobian.sum());
    EXPECT_EQ(-1, augJaco.jacobian(0, 0));
    VectorDynamic rhsExpect = GenerateVectorDynamic(4);
    rhsExpect << 0, -10, 0, 0;
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    augJaco.print();
}

TEST_F(DAGScheduleOptimizerTest1, GetExecutionTime)
{
    JobCEC job(2, 0);
    EXPECT_EQ(3, GetExecutionTime(job, tasksInfo));
}
// TODO: re-order variables?
// TODO: test cases when 's' and 'f' are interleaved
TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder)
{
    auto augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
    jacobianExpect << 1, -1, 0, 0,
        0, 1, 0, -1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(3);
    rhsExpect << -1, -1, -3;

    augJaco.print();

    // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

std::vector<std::vector<JobCEC>> SortJobsEachProcessor(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
{
    const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor(processorNum);
    // jobsOrderedEachProcessor.reserve(processorNum);
    for (uint i = 0; i < instanceOrder.size(); i++)
    {
        TimeInstance instCurr = instanceOrder[i];
        if (instCurr.type == 's') // only exam start instances
        {
            JobCEC &jobCurr = instCurr.job;
            LLint uniqueJobId = GetJobUniqueId(instCurr.job, tasksInfo);
            jobsOrderedEachProcessor[processorJobVec[uniqueJobId]].push_back(jobCurr);
        }
    }

    return jobsOrderedEachProcessor;
}
TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_single_core)
{
    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    EXPECT_EQ(4, jobsOrderedEachProcessor[0].size());
    EXPECT_EQ(0, jobsOrderedEachProcessor[0][0].taskId);
    EXPECT_EQ(0, jobsOrderedEachProcessor[0][1].taskId);
    EXPECT_EQ(2, jobsOrderedEachProcessor[0][2].taskId);
    EXPECT_EQ(1, jobsOrderedEachProcessor[0][3].taskId);
}

TEST_F(DAGScheduleOptimizerTest1, SortJobsEachProcessor_multi_core)
{
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    EXPECT_EQ(3, jobsOrderedEachProcessor[0].size());
    EXPECT_EQ(1, jobsOrderedEachProcessor[1].size());
    EXPECT_EQ(1, jobsOrderedEachProcessor[1][0].taskId);
}

// jobOrder's instanceOrder_ may be updated
AugmentedJacobian GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
{
    // find all the jobs that are executed on the same processor

    // For each processor, sort all the jobs' execution order

    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
    // Add DBF constraints for each adjacent job pair

    int m = tasksInfo.length - 1;
    MatrixDynamic jacobian = GenerateMatrixDynamic(m, tasksInfo.length);
    VectorDynamic rhs = GenerateVectorDynamic(m);
    int count = 0;
    const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
    for (uint i = 1; i < instanceOrder.size(); i++)
    {
        auto instCurr = instanceOrder[i];
        auto instPrev = instanceOrder[i - 1];
        if (instPrev.job == instCurr.job)
            continue;
        int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo);
        int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo);

        if (count > 2)
            CoutError("Please provide implementation for discontinuous 's' and 'f' in GetJacobianJobOrder");
        jacobian(count, globalIdCurr) = -1;
        jacobian(count, globalIdPrev) = 1;
        if (instPrev.type == 's')
        {
            if (instCurr.type == 's')
            {
                rhs(count) = 0;
            }
            else // instCurr.type == 'f'
            {
                rhs(count) = GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        else // instPrev.type == 'f'
        {
            if (instCurr.type == 's')
            {
                rhs(count) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
            }
            else // type == 'f'
            {
                rhs(count) = -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        count++;
    }

    return AugmentedJacobian{jacobian, rhs};
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDBF)
{
    auto augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
    jacobianExpect << 1, -1, 0, 0,
        0, 1, 0, -1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(3);
    rhsExpect << -1, -1, -3;

    augJaco.print();

    // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}