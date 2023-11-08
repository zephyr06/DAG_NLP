#pragma once

#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/OrderScheduler.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        // the time that the instance happens must be larger than the return value
        double GetInstanceLeastStartTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo);

        // the time that the instance happens must be smaller than the return value
        double GetInstanceMaxFinishTime(const TimeInstance &instance, const TaskSetInfoDerived &tasksInfo);
        
        bool WhetherSkipInsertStart(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);

        bool WhetherSkipInsertStartByPreviousInstance(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);

        bool WhetherSkipInsertStartByFollowingInstance(const JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);

        bool WhetherSkipInsertFinish(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);
        
        bool WhetherSkipInsertFinishByPreviousInstance(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);
        
        bool WhetherSkipInsertFinishByFollowingInstance(const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr);

        // This version is safer than the last version
        bool WhetherStartFinishTooLong(double &accumLengthMin, const JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrderCurrForStart, LLint startP);

        // TODO: test this function
        bool SubGroupSchedulabilityCheck(JobGroupRange &jobGroup, SFOrder &jobOrderRef, const SFOrder &jobOrderCurrForFinish, LLint finishP, DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, int processorNum);
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
