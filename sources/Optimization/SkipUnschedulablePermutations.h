#pragma once

#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/OrderScheduler.h"

namespace OrderOptDAG_SPACE
{
  namespace OptimizeSF
    {
    template <typename OrderScheduler>
    std::vector<int> FindLongestChainJobIndex(IterationStatus<OrderScheduler> &status)
    {
        std::vector<int> index(status.rtdaVec_.size(), 0);
        for (uint i = 0; i < status.rtdaVec_.size(); i++) // for each chain
        {
            std::vector<RTDA> &rtdaVec = status.rtdaVec_[i];
            auto ite = std::max_element(rtdaVec.begin(), rtdaVec.end(), [](RTDA r1, RTDA r2)
                                        { return ObjRTDA(r1) < ObjRTDA(r2); });
            index[i] = std::distance(rtdaVec.begin(), ite);
        }
        return index;
    }

    // the time that the instance happens must be larger than the return value
    double GetInstanceLeastStartTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo)
    {
        double prevInstanceLeastFinishTime = GetActivationTime(instance.job, tasksInfo);
        if (instance.type == 'f')
            prevInstanceLeastFinishTime += GetExecutionTime(instance.job, tasksInfo);
        return prevInstanceLeastFinishTime;
    }

    // the time that the instance happens must be smaller than the return value
    double GetInstanceMaxFinishTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo)
    {
        double nextInstanceLeastFinishTime = GetDeadline(instance.job, tasksInfo);
        if (instance.type == 's')
            nextInstanceLeastFinishTime -= tasksInfo.tasks[instance.job.taskId].executionTime;
        return nextInstanceLeastFinishTime;
    }
    bool WhetherSkipInsertStart(JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
    {
        if (startP > 0)
        {
            TimeInstance instancePrev = jobOrderCurr.instanceOrder_[startP - 1];
            // jP.ActivationTime <= jR.start <= jR.deadline - jR.executionTime
            double prevInstanceLeastFinishTime = GetInstanceLeastStartTime(instancePrev, tasksInfo);
            if (prevInstanceLeastFinishTime > GetDeadline(jobRelocate, tasksInfo) - tasksInfo.tasks[jobRelocate.taskId].executionTime)
                return true;
        }
        if (startP < tasksInfo.length * 2 - 1)
        {
            TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[startP + 1];
            //  jR.ActivationTime <= jR.start <= nextJ.Deadline
            double nextInstanceLeastFinishTime = GetInstanceMaxFinishTime(instanceAfter, tasksInfo);
            if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                return true;
        }
        return false;
    }

    bool WhetherSkipInsertFinish(JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
    {
        if (finishP > 0)
        {
            TimeInstance instancePrev = jobOrderCurr.instanceOrder_[finishP - 1];
            // jP.ActivationTime <= jR.finish <= jR.deadline
            double prevInstanceLeastFinishTime = GetInstanceLeastStartTime(instancePrev, tasksInfo);
            if (prevInstanceLeastFinishTime > GetDeadline(jobRelocate, tasksInfo))
                return true;
        }
        if (finishP < tasksInfo.length * 2 - 1)
        {
            TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[finishP + 1];
            //  jR.ActivationTime <= jR.finish <= nextJ.Deadline
            double nextInstanceLeastFinishTime = GetInstanceMaxFinishTime(instanceAfter, tasksInfo);
            if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                return true;
        }
        return false;
    }
    bool WhetherStartFinishTooLong(double &accumLengthMin, JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurrForStart, LLint startP)
    {
        if (accumLengthMin >= tasksInfo.tasks[jobRelocate.taskId].executionTime)
            return true;
        if (finishP < jobOrderCurrForStart.size())
        {
            TimeInstance jobPrevInsertInst = jobOrderCurrForStart.at(finishP);
            if (jobPrevInsertInst.type == 'f')
                accumLengthMin += tasksInfo.tasks[jobPrevInsertInst.job.taskId].executionTime;
        }
        return false;
    }

    template <typename OrderScheduler>
    bool SubGroupSchedulabilityCheck(JobGroupRange &jobGroup, IterationStatus<OrderScheduler> &statusPrev, SFOrder &jobOrderCurrForFinish, LLint finishP, DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, int processorNum)
    {
        if (enableSmallJobGroupCheck)
        {
            BeginTimer("FindUnschedulableSmallJobOrder");
            JobCEC jobNewlyAdded = jobOrderCurrForFinish[finishP - 1].job;
            jobGroup.minIndex = min(jobGroup.minIndex, statusPrev.jobOrder_.GetJobStartInstancePosition(jobNewlyAdded));

            jobGroup.maxIndex = max(jobGroup.maxIndex, finishP);
            jobGroup.maxIndex = max(jobGroup.maxIndex, statusPrev.jobOrder_.GetJobFinishInstancePosition(jobNewlyAdded) + 1);
            jobGroup.maxIndex = min(jobGroup.maxIndex, statusPrev.jobOrder_.size());
            jobGroup.minIndex = max(jobGroup.minIndex, jobGroup.maxIndex - subJobGroupMaxSize);
            jobGroup.minIndex = max(jobGroup.minIndex, 0);
            // countSubJobOrderLength.push_back(jobGroup.maxIndex - jobGroup.minIndex);
            std::vector<TimeInstance> instanceOrderSmall = ExtractSubInstances(jobOrderCurrForFinish, jobGroup);

            // BeginTimer("PrevSchedulabilityCheck");
            // bool bigFail = false;
            // if (SFOrderScheduling(dagTasks, tasksInfo, processorNum, jobOrderCurrForFinish)(0) == -1)
            // {
            //     bigFail = true;
            //     if (bigJobGroupCheck)
            //     {
            //         break;
            //     }
            // }
            // EndTimer("PrevSchedulabilityCheck");

            SFOrder jobOrderSmall(tasksInfo, instanceOrderSmall);

            bool smallFail = SFOrderScheduling(dagTasks, tasksInfo, processorNum, jobOrderSmall)(0) == -1;
            // if (bigFail == false && smallFail == true)
            // {
            //     if (debugMode == 1)
            //         jobOrderSmall.print();
            //     // CoutWarning("One mismatched group check!");
            // }

            EndTimer("FindUnschedulableSmallJobOrder");
            if (smallFail)
                return true;
        }

        return false;
    }
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
