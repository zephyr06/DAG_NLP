

#include "sources/Optimization/JacobianAnalyze.h"

namespace OrderOptDAG_SPACE
{
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

    // GetDAGJacobianOrg
    AugmentedJacobian GetDAGJacobianOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum)
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

    std::unordered_map<JobCEC, size_t> GetJobOrderMap(const SFOrder &jobOrder)
    {
        int jobIndex = 0;
        std::unordered_map<JobCEC, size_t> jobIndexInJacobian;
        const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
        for (uint i = 0; i < instanceOrder.size(); i++)
        {
            if (instanceOrder[i].type == 's')
            {
                jobIndexInJacobian[instanceOrder[i].job] = jobIndex++;
            }
        }
        return jobIndexInJacobian;
    }

    VectorDynamic ReOrderLPObj(const VectorDynamic &c, const SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo)
    {
        std::unordered_map<JobCEC, size_t> jobIndexInJacobian = GetJobOrderMap(jobOrder);
        VectorDynamic cOrdered(tasksInfo.length);
        int count = 0;
        for (uint i = 0; i < tasksInfo.tasks.size(); i++)
        {
            for (int j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC jobCurr(i, j);
                cOrdered(jobIndexInJacobian[jobCurr]) = c(count++);
            }
        }
        return cOrdered;
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

            int jacobianIndexToUpdate = std::min(globalIdPrev, globalIdCurr);
            UpdateAugmentedJacobianJobOrder(jacobs[jacobianIndexToUpdate], instPrev, instCurr, rowCount[jacobianIndexToUpdate], globalIdPrev, globalIdCurr, tasksInfo);
            rowCount[jacobianIndexToUpdate]++;
        }

        jobIndex = 0;
        for (auto &augJacob : jacobs)
        {
            augJacob.jacobian.conservativeResize(rowCount[jobIndex], n);
            augJacob.rhs.conservativeResize(rowCount[jobIndex++], 1);
        }

        return jacobs;
    }

    // TODO: this function could possibly improve efficiency?
    // TODO: switch Jacobian to sparseMatrix
    AugmentedJacobian MergeAugJacobian(const std::vector<AugmentedJacobian> &augJacos)
    {
        BeginTimer("MergeAugJacobian");
        AugmentedJacobian jacobAll;
        if (augJacos.size() == 0)
            return jacobAll;

        int n = augJacos[0].jacobian.cols();
        int totalRow = 0;
        for (uint i = 0; i < augJacos.size(); i++)
            totalRow += augJacos[i].jacobian.rows();

        jacobAll.jacobian.conservativeResize(totalRow, n);
        jacobAll.rhs.conservativeResize(totalRow, 1);
        int rowCount = 0;
        for (uint i = 0; i < augJacos.size(); i++)
        {
            jacobAll.jacobian.block(rowCount, 0, augJacos[i].jacobian.rows(), n) = augJacos[i].jacobian;
            jacobAll.rhs.block(rowCount, 0, augJacos[i].rhs.rows(), 1) = augJacos[i].rhs;
            rowCount += augJacos[i].jacobian.rows();
        }
        EndTimer("MergeAugJacobian");
        return jacobAll;
    }

    // Each one or two rows correspond to one chain, first RT constraint, then DA constraint;
    // if DA data is overwritten, then we'll skip this row;
    // As for the added variable, it's also RT first DA second;
    // jobOrder's jobSFMap_ may be updated
    // TODO: this function only considres 1 chain, add functiosn for multiple chains
    AugmentedJacobian GetJacobianCauseEffectChainOrg(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, const std::vector<uint> processorJobVec, int processorNum, const std::vector<int> &causeEffectChain, int chainIndex)
    {
        std::unordered_map<JobCEC, JobCEC> firstReactionMap;

        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        LLint totalStartJobs = hyperPeriod / tasks[causeEffectChain[0]].period + 1;

        AugmentedJacobian augJacob;
        int varNum = tasksInfo.length + 2 * (1 + chainIndex);
        augJacob.jacobian.conservativeResize((totalStartJobs + 1) * 2, varNum);
        augJacob.rhs.conservativeResize((totalStartJobs + 1) * 2, 1);
        int varIndexRT = tasksInfo.length + 2 * (chainIndex);
        int varIndexDA = tasksInfo.length + 1 + 2 * (chainIndex);

        if (causeEffectChain.size() == 0)
        {
            return augJacob;
        }

        int rowIndex = 0;
        for (LLint startInstanceIndex = 0; startInstanceIndex <= totalStartJobs; startInstanceIndex++)
        {
            JobCEC firstJob = {causeEffectChain[0], (startInstanceIndex)};
            std::vector<JobCEC> react_chain;
            react_chain.push_back(firstJob);
            for (uint j = 1; j < causeEffectChain.size(); j++)
            {
                LLint instIndexFirstJob = jobOrder.GetJobFinishInstancePosition(firstJob);
                LLint jobIndex = 0;
                while (true)
                {
                    JobCEC jobCurr{causeEffectChain[j], jobIndex};
                    if (jobOrder.GetJobStartInstancePosition(jobCurr) > instIndexFirstJob)
                        break;
                    jobIndex++;
                    if (jobIndex > 10000)
                    {
                        CoutError("didn't find a match in GetJacobianCauseEffectChainOrg!");
                    }
                }
                firstJob = {causeEffectChain[j], jobIndex};
                react_chain.push_back(firstJob);
            }

            JobCEC jj(causeEffectChain[0], startInstanceIndex);
            firstReactionMap[jj] = firstJob;

            // update augJacob
            augJacob.jacobian.row(rowIndex).setZero();
            int sourceJobCol = GetJobUniqueId(jj, tasksInfo);
            int tailJobCol = GetJobUniqueId(firstJob, tasksInfo);
            augJacob.jacobian(rowIndex, tailJobCol) = 1;
            augJacob.jacobian(rowIndex, sourceJobCol) = -1;
            augJacob.jacobian(rowIndex, sourceJobCol) = -1;
            augJacob.jacobian(rowIndex, varIndexRT) = -1;
            augJacob.rhs(rowIndex) = -1 * GetExecutionTime(firstJob, tasksInfo);
            rowIndex++;

            // update data age
            JobCEC firstReactLastJob = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
            if (startInstanceIndex > 0 && firstReactionMap[firstReactLastJob] != firstJob && firstJob.jobId > 0)
            {
                JobCEC lastReaction = firstJob;
                lastReaction.jobId--;
                // resVec[startInstanceIndex - 1].dataAge = GetStartTime(lastReaction, x, tasksInfo) + tasks[lastReaction.taskId].executionTime - GetStartTime({causeEffectChain[0], startInstanceIndex - 1}, x, tasksInfo);
                augJacob.jacobian.row(rowIndex).setZero();
                JobCEC sourceJobDA = JobCEC{causeEffectChain[0], (startInstanceIndex - 1)};
                int sourceJobColDA = GetJobUniqueIdWithinHyperPeriod(sourceJobDA, tasksInfo);
                int tailJobColDA = GetJobUniqueIdWithinHyperPeriod(lastReaction, tasksInfo);
                augJacob.jacobian(rowIndex, tailJobColDA) = 1;
                augJacob.jacobian(rowIndex, sourceJobColDA) = -1;
                augJacob.jacobian(rowIndex, varIndexDA) = -1;
                augJacob.rhs(rowIndex) = -1 * GetExecutionTime(lastReaction, tasksInfo);
                rowIndex++;
            }
        }
        augJacob.jacobian.conservativeResize(rowIndex, varNum);
        augJacob.rhs.conservativeResize(rowIndex, 1);
        return augJacob;
    }

} // namespace OrderOptDAG_SPACE