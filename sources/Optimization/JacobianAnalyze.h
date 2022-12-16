
#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Utils/profilier.h"

namespace OrderOptDAG_SPACE
{

    // TODO: work with SM_Matrix
    struct AugmentedJacobian
    {
        MatrixDynamic jacobian;
        VectorDynamic rhs;

        AugmentedJacobian() {}

        AugmentedJacobian(MatrixDynamic jacobian,
                          VectorDynamic rhs) : jacobian(jacobian), rhs(rhs) {}

        AugmentedJacobian(int m, int n) : jacobian(GenerateMatrixDynamic(m, n)), rhs(GenerateVectorDynamic(m)) {}

        void print()
        {
            std::cout << "Jacobian: " << jacobian << std::endl;
            std::cout << "RHS: " << rhs << std::endl;
        }
    };

    // Warning! This function is slow because it creates a new AugmentedJacobian object
    AugmentedJacobian StackAugJaco(const AugmentedJacobian &augJaco1, const AugmentedJacobian &augJaco2);

    AugmentedJacobian GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo);

    AugmentedJacobian GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo);

    AugmentedJacobian GetDAGJacobianOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum);

    std::vector<std::vector<JobCEC>> SortJobsEachProcessor(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum);

    AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder);

    AugmentedJacobian GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum);

    std::unordered_map<JobCEC, size_t> GetJobOrderMap(const SFOrder &jobOrder);

    // This function requires more consideration
    // order of AugmentedJacobian follows instanceOrder in jobOrder
    // The columns of each Jacobian matrix follows instanceOrder in jobOrder
    // TODO: clean code, refactor function
    // return: all the Jacobian matrices, columns and rows are re-ordered following jobs' dispatch order
    std::vector<AugmentedJacobian> GetVariableBlocks(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum);

    // Input is the original coefficient vector c for LP without "band" re-ordering trick
    VectorDynamic ReOrderLPObj(const VectorDynamic &c, const SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo);

    AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos);

    inline AugmentedJacobian GetDAGJacobianOrdered(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
    {
        std::vector<AugmentedJacobian> augJacobs = GetVariableBlocks(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
        return MergeAugJacobian(augJacobs);
    }

    AugmentedJacobian GetJacobianCauseEffectChainOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum, const std::vector<int> &causeEffectChain);

} // namespace OrderOptDAG_SPACE