

#include "sources/Optimization/JacobianAnalyze.h"

namespace OrderOptDAG_SPACE
{
    // TODO: improve efficiency for this function
    AugmentedJacobian StackAugJaco(const AugmentedJacobian &augJaco1, const AugmentedJacobian &augJaco2)
    {
        MatrixDynamic jacobianAll(augJaco1.jacobian.rows() + augJaco2.jacobian.rows(), augJaco1.jacobian.cols());
        jacobianAll << augJaco1.jacobian, augJaco2.jacobian;

        VectorDynamic rhsAll(augJaco1.rhs.rows() + augJaco2.rhs.rows(), 1);
        rhsAll << augJaco1.rhs, augJaco2.rhs;
        return AugmentedJacobian{jacobianAll, rhsAll};
    }

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

    AugmentedJacobian GetJacobianAll(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
    {
        AugmentedJacobian augJacoDDL = GetJacobianDDL(dagTasks, tasksInfo);
        AugmentedJacobian augJacoAct = GetJacobianActivationTime(dagTasks, tasksInfo);
        AugmentedJacobian augJacoDBF = GetJacobianDBF(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
        AugmentedJacobian augJacoJobOrder = GetJacobianJobOrder(dagTasks, tasksInfo, jobOrder);

        AugmentedJacobian augJacobAll = StackAugJaco(augJacoDDL, augJacoAct);
        augJacobAll = StackAugJaco(augJacobAll, augJacoDBF);
        augJacobAll = StackAugJaco(augJacobAll, augJacoJobOrder);
        return augJacobAll;
    }

    // For each processor, the jobs are ordered following the gien jobOrder
    std::vector<std::vector<JobCEC>> SortJobsEachProcessor(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
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

        // remove idle processor
        int i = jobsOrderedEachProcessor.size() - 1;
        for (; i >= 0; i--)
        {
            if (jobsOrderedEachProcessor[i].empty())
                jobsOrderedEachProcessor.erase(jobsOrderedEachProcessor.begin() + i);
        }

        return jobsOrderedEachProcessor;
    }

    AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder)
    {
        MatrixDynamic jacobian;
        jacobian.conservativeResize(tasksInfo.length * 2 - 1, tasksInfo.length);
        jacobian.setZero();
        VectorDynamic rhs;
        rhs.conservativeResize(tasksInfo.length * 2 - 1, 1);
        rhs.setZero();

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

        jacobian.conservativeResize(count, tasksInfo.length);
        rhs.conservativeResize(count, 1);
        return AugmentedJacobian{jacobian, rhs};
    }
    AugmentedJacobian GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
    {
        // find all the jobs that are executed on the same processor

        // For each processor, sort all the jobs' execution order

        std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
        // Add DBF constraints for each adjacent job pair

        int m = tasksInfo.length - 1 - (static_cast<int>(jobsOrderedEachProcessor.size()) - 1);
        MatrixDynamic jacobian = GenerateMatrixDynamic(m, tasksInfo.length);
        VectorDynamic rhs = GenerateVectorDynamic(m);
        int count = 0;
        for (uint processorId = 0; processorId < jobsOrderedEachProcessor.size(); processorId++)
        {
            const std::vector<JobCEC> &jobsOrdered = jobsOrderedEachProcessor[processorId];
            for (uint i = 1; i < jobsOrdered.size(); i++)
            {
                int globalIdPrev = GetJobUniqueId(jobsOrdered.at(i - 1), tasksInfo);
                int globalIdCurr = GetJobUniqueId(jobsOrdered.at(i), tasksInfo);
                jacobian(count, globalIdPrev) = 1;
                jacobian(count, globalIdCurr) = -1;
                rhs(count) = -1 * GetExecutionTime(jobsOrdered.at(i - 1), tasksInfo);
                count++;
            }
        }

        return AugmentedJacobian{jacobian, rhs};
    }
} // namespace OrderOptDAG_SPACE