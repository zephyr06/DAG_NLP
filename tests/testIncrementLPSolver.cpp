#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Jacobi>
#include "gtsam/base/Testable.h" // assert_equal

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/JacobianAnalyze.h"
#include "sources/Utils/IncrementQR.h"

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

TEST_F(DAGScheduleOptimizerTest1, GetJacobianDBF)
{
    auto augJaco = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(3, 4);
    jacobianExpect << 1, -1, 0, 0,
        0, 1, 0, -1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(3);
    rhsExpect << -1, -1, -3;
    augJaco.print();
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect, augJaco.rhs));
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, GetJacobianJobOrder_non_continuous_s_f)
{
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    auto augJaco = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(5, 4);
    jacobianExpect << 1, 0, -1, 0,
        -1, 0, 1, 0,
        1, 0, -1, 0,
        0, -1, 1, 0,
        0, 1, 0, -1;
    VectorDynamic rhsExpect = GenerateVectorDynamic(5);
    rhsExpect << 0, 1, 1, -2, -1;

    augJaco.print();

    // EXPECT_EQ(jacobianExpect, augJaco.jacobian);
    EXPECT_EQ(rhsExpect, augJaco.rhs);
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect, augJaco.jacobian));
}

TEST_F(DAGScheduleOptimizerTest1, StackAugJaco)
{
    AugmentedJacobian augJacoDDL = GetJacobianDDL(dagTasks, tasksInfo);
    AugmentedJacobian augJacoAct = GetJacobianActivationTime(dagTasks, tasksInfo);
    AugmentedJacobian augJacoDBF = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

    AugmentedJacobian augJacobAll = StackAugJaco(augJacoDDL, augJacoAct);
    augJacobAll = StackAugJaco(augJacobAll, augJacoDBF);
    augJacobAll.print();
    EXPECT_EQ(4 + 4 + 3, augJacobAll.jacobian.rows());
    EXPECT_EQ(4 + 4 + 6, augJacobAll.jacobian.squaredNorm());
    EXPECT_EQ(20 + 20 + 20 - 1 - 1 - 3, augJacobAll.rhs.sum());
}
TEST_F(DAGScheduleOptimizerTest1, GetJacobianAll)
{
    AugmentedJacobian augJacobAll = GetJacobianAll(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    augJacobAll.print();

    EXPECT_EQ(4 + 4 + 3 + 3, augJacobAll.jacobian.rows());
    EXPECT_EQ(4 + 4 + 6 + 6, augJacobAll.jacobian.squaredNorm());
    EXPECT_EQ(20 + 20 + 20 - 1 - 1 - 3 - 1 - 1 - 3, augJacobAll.rhs.sum());
}

// This function requires more consideration
// order of AugmentedJacobian follows instanceOrder in jobOrder
// The columns of each Jacobian matrix follows instanceOrder in jobOrder
std::vector<AugmentedJacobian> GetVariableBlocks(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
{
    int n = tasksInfo.length; // number of variables
    int m = 4;                // rows of Jacobian in each AugmentedJacobian

    // prepare the results initialization
    AugmentedJacobian jacobRef(m, n);
    jacobRef.jacobian.conservativeResize(4 + n, n);
    std::vector<AugmentedJacobian> jacobs;
    jacobs.reserve(n);
    for (int i = 0; i < n; i++)
        jacobs.push_back(jacobRef);

    std::vector<int> rowCount(n);
    const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;

    std::unordered_map<JobCEC, size_t> jobIndexInJacobian;
    int jobIndex = 0;
    for (uint i = 0; i < instanceOrder.size(); i++)
    {
        auto instCurr = instanceOrder[i];
        JobCEC jobCurr = instCurr.job;
        if (instCurr.type == 's')
        {
            // set DDL
            jacobs[jobIndex].jacobian(0, jobIndex) = 1;
            jacobs[jobIndex].rhs(0) = GetDeadline(jobCurr, tasksInfo);

            // set Activation
            jacobs[jobIndex].jacobian(1, jobIndex) = -1;
            jacobs[jobIndex].rhs(1) = -1 * GetActivationTime(jobCurr, tasksInfo);

            jobIndexInJacobian[jobCurr] = jobIndex;
            rowCount[jobIndex] = 2;
            jobIndex++;
        }
    }

    // set DBF
    // TODO: clean code, probably create a struct for rowCount, jacobs, etc
    std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);

    for (uint processorId = 0; processorId < jobsOrderedEachProcessor.size(); processorId++)
    {
        const std::vector<JobCEC> &jobsOrdered = jobsOrderedEachProcessor[processorId];
        for (uint i = 1; i < jobsOrdered.size(); i++)
        {
            int globalIdPrev = jobIndexInJacobian[jobsOrdered.at(i - 1)];
            int globalIdCurr = jobIndexInJacobian[jobsOrdered.at(i)];
            // let's assume that globalIdPrev happens earlier than globalIdCurr, which is actually guaranteed by SortJobsEachProcessor(...)
            jacobs[globalIdPrev].jacobian(rowCount[globalIdPrev], globalIdPrev) = 1;
            jacobs[globalIdPrev].jacobian(rowCount[globalIdPrev], globalIdCurr) = -1;
            jacobs[globalIdPrev].rhs(rowCount[globalIdPrev]) = -1 * GetExecutionTime(jobsOrdered.at(i - 1), tasksInfo);
            rowCount[globalIdPrev]++;
        }
    }

    // set job order
    for (uint i = 1; i < instanceOrder.size(); i++)
    {
        auto instCurr = instanceOrder[i];
        auto instPrev = instanceOrder[i - 1];

        if (instPrev.job == instCurr.job)
            continue; // setting jacobian and rhs as 0 vectors
        int globalIdPrev = jobIndexInJacobian[instPrev.job];
        int globalIdCurr = jobIndexInJacobian[instCurr.job];

        jacobs[globalIdPrev].jacobian(rowCount[globalIdPrev], globalIdPrev) = 1;
        jacobs[globalIdPrev].jacobian(rowCount[globalIdPrev], globalIdCurr) = -1;
        if (instPrev.type == 's')
        {
            if (instCurr.type == 's')
            {
                jacobs[globalIdPrev].rhs(rowCount[globalIdPrev]) = 0;
                // rhs(jobIndex) = 0;
            }
            else // instCurr.type == 'f'
            {
                // rhs(jobIndex) = GetExecutionTime(instCurr.job, tasksInfo);
                jacobs[globalIdPrev].rhs(rowCount[globalIdPrev]) = GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        else // instPrev.type == 'f'
        {
            if (instCurr.type == 's')
            {
                // rhs(jobIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
                jacobs[globalIdPrev].rhs(rowCount[globalIdPrev]) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
            }
            else // type == 'f'
            {
                // rhs(jobIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
                jacobs[globalIdPrev].rhs(rowCount[globalIdPrev]) = -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        rowCount[globalIdPrev]++;
    }

    jobIndex = 0;
    for (auto &augJacob : jacobs)
    {
        augJacob.jacobian.conservativeResize(rowCount[jobIndex++], n);
    }

    return jacobs;
}
TEST_F(DAGScheduleOptimizerTest1, GetVariableBlocks)
{
    std::vector<AugmentedJacobian> augJacos = GetVariableBlocks(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

    MatrixDynamic jacobianExpect0(4, 4);
    jacobianExpect0 << 1, 0, 0, 0,
        -1, 0, 0, 0,
        1, -1, 0, 0,
        1, -1, 0, 0;
    VectorDynamic rhsExpect0(4);
    rhsExpect0 << 10, 0, -1, -1;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect0, augJacos[0].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect0, augJacos[0].rhs));

    MatrixDynamic jacobianExpect1(4, 4);
    jacobianExpect1 << 0, 1, 0, 0,
        0, -1, 0, 0,
        0, 1, -1, 0,
        0, 1, -1, 0;
    VectorDynamic rhsExpect1(4);
    rhsExpect1 << 20, -10, -1, -1;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect1, augJacos[1].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect1, augJacos[1].rhs));

    MatrixDynamic jacobianExpect2(4, 4);
    jacobianExpect2 << 0, 0, 1, 0,
        0, 0, -1, 0,
        0, 0, 1, -1,
        0, 0, 1, -1;
    VectorDynamic rhsExpect2(4);
    rhsExpect2 << 20, 0, -3, -3;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect2, augJacos[2].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect2, augJacos[2].rhs));

    MatrixDynamic jacobianExpect3(2, 4);
    jacobianExpect3 << 0, 0, 0, 1,
        0, 0, 0, -1;
    VectorDynamic rhsExpect3(4);
    rhsExpect3 << 20, 0, 0, 0;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect3, augJacos[3].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect3, augJacos[3].rhs));
}

TEST_F(DAGScheduleOptimizerTest1, GetVariableBlock_non_continuous)
{
    initial = GenerateVectorDynamic(4);
    initial << 0, 10, 0, 12;
    jobOrder = SFOrder(tasksInfo, initial);
    jobOrder.print();
    VectorDynamic _ = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
    AugmentedJacobian augJacobAll = GetJacobianAll(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
    augJacobAll.print();

    std::vector<AugmentedJacobian> augJacos = GetVariableBlocks(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);

    MatrixDynamic jacobianExpect0(4, 4);
    jacobianExpect0 << 1, 0, 0, 0,
        -1, 0, 0, 0,
        1, -1, 0, 0,
        1, -1, 0, 0;
    VectorDynamic rhsExpect0(4);
    rhsExpect0 << 10, 0, -1, -1;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect0, augJacos[0].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect0, augJacos[0].rhs));

    MatrixDynamic jacobianExpect1(4, 4);
    jacobianExpect1 << 0, 1, 0, 0,
        0, -1, 0, 0,
        0, 1, 0, -1,
        0, 1, 0, -1;
    VectorDynamic rhsExpect1(4);
    rhsExpect1 << 20, -10, -1, -1;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect1, augJacos[1].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect1, augJacos[1].rhs));

    MatrixDynamic jacobianExpect2(4, 4);
    jacobianExpect2 << 0, 0, 1, 0,
        0, 0, -1, 0,
        0, 0, -1, 1,
        0, 0, -1, 1;
    VectorDynamic rhsExpect2(4);
    rhsExpect2 << 20, 0, -3, -3;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect2, augJacos[2].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect2, augJacos[2].rhs));

    MatrixDynamic jacobianExpect3(4, 4);
    jacobianExpect3 << 0, 0, 0, 1,
        0, 0, 0, -1,
        0, 0, 0, 0,
        0, 0, 0, 0;
    VectorDynamic rhsExpect3(4);
    rhsExpect3 << 20, 0, 0, 0;
    EXPECT_TRUE(gtsam::assert_equal(jacobianExpect3, augJacos[3].jacobian));
    EXPECT_TRUE(gtsam::assert_equal(rhsExpect3, augJacos[3].rhs));
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}