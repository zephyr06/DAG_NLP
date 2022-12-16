#include <gtest/gtest.h>
#include "gmock/gmock.h" // Brings in gMock.
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/JacobianAnalyze.h"
#include "sources/Utils/IncrementQR.h"
#include "sources/Optimization/LinearProgrammingSolver.h"
#include "sources/Optimization/LPSolverCplex.h"
#include "sources/Utils/profilier.h"

using namespace OrderOptDAG_SPACE;
using namespace OrderOptDAG_SPACE::OptimizeSF;
using namespace GlobalVariablesDAGOpt;
using namespace LPOptimizer;
class LPTest1 : public ::testing::Test
{
protected:
    void SetUp() override
    {
        dagTasks = ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/" + testDataSetName + ".csv", "orig", 1);
        tasks = dagTasks.tasks;
        tasksInfo = TaskSetInfoDerived(tasks);
        // chain1 = {0, 2};
        // dagTasks.chains_[0] = chain1;
        timeLimits = 10;
        scheduleOptions.processorNum_ = 2;
        scheduleOptions.considerSensorFusion_ = 0;
        scheduleOptions.freshTol_ = 0;
        scheduleOptions.sensorFusionTolerance_ = 0;
        scheduleOptions.weightInMpRTDA_ = 0.5;
        scheduleOptions.weightInMpSf_ = 0.5;
        scheduleOptions.weightPunish_ = 10;

        VectorDynamic initial = ListSchedulingLFTPA(dagTasks, tasksInfo, scheduleOptions.processorNum_, processorJobVec);
        // std::cout << "Initial solutions: " << std::endl
        //           << initial << std::endl
        //           << std::endl;
        jobOrder = SFOrder(tasksInfo, initial);
        // jobOrder.print();

        c = GenerateVectorDynamic(tasksInfo.length);
        c(0) = 1;
        c(1) = -1;

        AugmentedJacobian jacobAll;
        if (GlobalVariablesDAGOpt::ReOrderProblem == "orig")
            jacobAll = GetDAGJacobianOrg(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
        else if (GlobalVariablesDAGOpt::ReOrderProblem == "band")
        {
            jacobAll = GetDAGJacobianOrdered(dagTasks, tasksInfo, jobOrder, processorJobVec, scheduleOptions.processorNum_);
            c = ReOrderLPObj(c, jobOrder, tasksInfo);
        }

        else if (GlobalVariablesDAGOpt::ReOrderProblem == "amd")
            CoutError("Please provide reorder method implementation!");
        else
            CoutError("Please provide reorder method implementation!");

        A = jacobAll.jacobian;
        As = A.sparseView();

        b = jacobAll.rhs;
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

    MatrixDynamic A;
    Eigen::SparseMatrix<double> As;
    VectorDynamic b;
    VectorDynamic c;
};

TEST_F(LPTest1, SolveLP)
{
    TimerFunc _;
    BeginTimer("main");
    // VectorDynamic xExpect(2);
    // xExpect << 0, 15;
    VectorDynamic xActual = SolveLP(As, b, c);
    EndTimer("main");
    if (GlobalVariablesDAGOpt::PrintOutput)
        std::cout << "Solution found: \n"
                  << xActual << std::endl;
    EXPECT_THAT((b - As * xActual).minCoeff(), testing::Ge(-1e-4));
    std::cout << "The number of variables is: " << xActual.rows() << std::endl;
    std::cout << "Optimal solution found: " << c.transpose() * xActual << std::endl;

    PrintTimer();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}