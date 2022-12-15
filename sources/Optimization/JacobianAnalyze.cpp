

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

    inline void UpdateAugmentedJacobianDDL(AugmentedJacobian &augJacob, int rowIndex, int jobIndex, const JobCEC &jobCurr, const TaskSetInfoDerived &tasksInfo)
    {
        augJacob.jacobian(rowIndex, jobIndex) = 1;
        augJacob.rhs(rowIndex) = GetDeadline(jobCurr, tasksInfo) - GetExecutionTime(jobCurr, tasksInfo);
    }

    AugmentedJacobian GetJacobianDDL(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
    {
        AugmentedJacobian augJacob;
        augJacob.jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);
        augJacob.rhs = GenerateVectorDynamic(tasksInfo.length);

        int count = 0;
        for (uint i = 0; i < dagTasks.tasks.size(); i++)
        {
            for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC jobCurr((int)i, j);
                UpdateAugmentedJacobianDDL(augJacob, count, count, jobCurr, tasksInfo);
                count++;
            }
        }
        return augJacob;
    }

    inline void UpdateAugmentedJacobianActivationTime(AugmentedJacobian &augJacob, int rowIndex, int jobIndex, const JobCEC &jobCurr, const TaskSetInfoDerived &tasksInfo)
    {
        augJacob.jacobian(rowIndex, jobIndex) = -1;
        augJacob.rhs(rowIndex) = GetActivationTime(jobCurr, tasksInfo) * -1;
    }
    AugmentedJacobian GetJacobianActivationTime(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo)
    {
        AugmentedJacobian augJacob;
        augJacob.jacobian = GenerateMatrixDynamic(tasksInfo.length, tasksInfo.length);
        augJacob.rhs = GenerateVectorDynamic(tasksInfo.length);

        int count = 0;
        for (uint i = 0; i < dagTasks.tasks.size(); i++)
        {
            for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC jobCurr((int)i, j);
                UpdateAugmentedJacobianActivationTime(augJacob, count, count, jobCurr, tasksInfo);
                count++;
            }
        }
        return augJacob;
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
    // TODO: switch argument order of instCurr and instPrev
    void UpdateAugmentedJacobianJobOrder(AugmentedJacobian &augJacob, TimeInstance &instPrev, TimeInstance &instCurr, int rowIndex, int globalIdPrev, int globalIdCurr, const TaskSetInfoDerived &tasksInfo)
    {
        augJacob.jacobian.row(rowIndex).setZero();
        augJacob.jacobian(rowIndex, globalIdCurr) = -1;
        augJacob.jacobian(rowIndex, globalIdPrev) = 1;
        if (instPrev.type == 's')
        {
            if (instCurr.type == 's')
            {
                augJacob.rhs(rowIndex) = 0;
            }
            else // instCurr.type == 'f'
            {
                augJacob.rhs(rowIndex) = GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
        else // instPrev.type == 'f'
        {
            if (instCurr.type == 's')
            {
                augJacob.rhs(rowIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo);
            }
            else // type == 'f'
            {
                augJacob.rhs(rowIndex) = -1 * GetExecutionTime(instPrev.job, tasksInfo) + GetExecutionTime(instCurr.job, tasksInfo);
            }
        }
    }

    // Bug found: it can't correctly distinguish prev and next instances in cases such as (1,0,s) preceeds (0,0,f), but we'll probably not use this function anymore
    AugmentedJacobian GetJacobianJobOrder(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder)
    {
        AugmentedJacobian augJacob;
        augJacob.jacobian.conservativeResize(tasksInfo.length * 2 - 1, tasksInfo.length);
        augJacob.jacobian.setZero();
        augJacob.rhs.conservativeResize(tasksInfo.length * 2 - 1, 1);
        augJacob.rhs.setZero();

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
            UpdateAugmentedJacobianJobOrder(augJacob, instPrev, instCurr, count, globalIdPrev, globalIdCurr, tasksInfo);
            count++;
        }

        augJacob.jacobian.conservativeResize(count, tasksInfo.length);
        augJacob.rhs.conservativeResize(count, 1);
        return augJacob;
    }

    inline void UpdateAugmentedJacobianDBF(AugmentedJacobian &augJacob, int rowIndex, int globalIdPrev, int globalIdCurr, const JobCEC &jobPrev, const TaskSetInfoDerived &tasksInfo)
    {
        augJacob.jacobian(rowIndex, globalIdPrev) = 1;
        augJacob.jacobian(rowIndex, globalIdCurr) = -1;
        augJacob.rhs(rowIndex) = -1 * GetExecutionTime(jobPrev, tasksInfo);
    }

    AugmentedJacobian GetJacobianDBF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
    {
        // find all the jobs that are executed on the same processor

        // For each processor, sort all the jobs' execution order

        std::vector<std::vector<JobCEC>> jobsOrderedEachProcessor = SortJobsEachProcessor(dagTasks, tasksInfo, jobOrder, processorJobVec, processorNum);
        // Add DBF constraints for each adjacent job pair

        int m = tasksInfo.length - 1 - (static_cast<int>(jobsOrderedEachProcessor.size()) - 1);

        AugmentedJacobian augJacob;
        augJacob.jacobian = GenerateMatrixDynamic(m, tasksInfo.length);
        augJacob.rhs = GenerateVectorDynamic(m);
        int count = 0;
        for (uint processorId = 0; processorId < jobsOrderedEachProcessor.size(); processorId++)
        {
            const std::vector<JobCEC> &jobsOrdered = jobsOrderedEachProcessor[processorId];
            for (uint i = 1; i < jobsOrdered.size(); i++)
            {
                int globalIdPrev = GetJobUniqueId(jobsOrdered.at(i - 1), tasksInfo);
                int globalIdCurr = GetJobUniqueId(jobsOrdered.at(i), tasksInfo);
                UpdateAugmentedJacobianDBF(augJacob, count, globalIdPrev, globalIdCurr, jobsOrdered.at(i - 1), tasksInfo);
                count++;
            }
        }

        return augJacob;
    }

    std::vector<AugmentedJacobian> GetVariableBlocks(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
    {
        int n = tasksInfo.length; // number of variables
        int m = 4;                // rows of Jacobian in each AugmentedJacobian

        // prepare the results initialization
        AugmentedJacobian jacobRef(m, n);
        // maximum rows: 2 for DDL and Acti, 2 for DBF, 2 for JobOrder
        jacobRef.jacobian.conservativeResize(1 + 1 + 2 + 2, n);
        jacobRef.rhs.conservativeResize(1 + 1 + 2 + 2, 1);
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
                UpdateAugmentedJacobianDDL(jacobs[jobIndex], 0, jobIndex, jobCurr, tasksInfo);

                UpdateAugmentedJacobianActivationTime(jacobs[jobIndex], 1, jobIndex, jobCurr, tasksInfo);

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
                const JobCEC &jobPrev = jobsOrdered.at(i - 1);
                int globalIdPrev = jobIndexInJacobian[jobPrev];
                int globalIdCurr = jobIndexInJacobian[jobsOrdered.at(i)];
                // // let's assume that globalIdPrev happens earlier than globalIdCurr, which is actually guaranteed by SortJobsEachProcessor(...)
                UpdateAugmentedJacobianDBF(jacobs[globalIdPrev], rowCount[globalIdPrev], globalIdPrev, globalIdCurr, jobPrev, tasksInfo);
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
            if (globalIdPrev > globalIdCurr) // make sure that prev inst happens earlier than next inst
            {
                auto instTemp = instCurr;
                instCurr = instPrev;
                instPrev = instTemp;
                globalIdPrev = jobIndexInJacobian[instPrev.job];
                globalIdCurr = jobIndexInJacobian[instCurr.job];
            }

            UpdateAugmentedJacobianJobOrder(jacobs[globalIdPrev], instPrev, instCurr, rowCount[globalIdPrev], globalIdPrev, globalIdCurr, tasksInfo);
            rowCount[globalIdPrev]++;
        }

        jobIndex = 0;
        for (auto &augJacob : jacobs)
        {
            augJacob.jacobian.conservativeResize(rowCount[jobIndex], n);
            augJacob.rhs.conservativeResize(rowCount[jobIndex++], 1);
        }

        return jacobs;
    }

    // TODO: this function could improve efficiency
    AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos)
    {
        AugmentedJacobian jacobAll;
        if (augJacos.size() == 0)
            return jacobAll;

        int totalRow = 0;
        for (uint i = 0; i < augJacos.size(); i++)
            totalRow += augJacos[i].jacobian.rows();
        jacobAll = augJacos[0];
        // jacobAll.jacobian.conservativeResize(totalRow, augJacos[0].jacobian.cols());
        // jacobAll.rhs.conservativeResize(totalRow, 1);
        for (uint i = 1; i < augJacos.size(); i++)
        {
            jacobAll = StackAugJaco(jacobAll, augJacos[i]);
        }
        return jacobAll;
    }
} // namespace OrderOptDAG_SPACE