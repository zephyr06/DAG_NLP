#pragma once
#include "unordered_set"
#include "unordered_map"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Utils/Parameters.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Optimization/ScheduleOptimizer.h"
// #include "sources/Optimization/InitialEstimate.h"

namespace OrderOptDAG_SPACE
{
    class OrderScheduler
    {
    public:
        // jobOrder is allowed to change
        OrderScheduler() {}
        static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, const std::vector<uint> processorJobVec)
        {
            CoutError("Never call base function!");
            return GenerateVectorDynamic1D(0);
        }
    };

    class SimpleOrderScheduler : public OrderScheduler
    {
    public:
        // processorJobVec will be assigned values
        static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, std::vector<uint> &processorJobVec)
        {
            return SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
        }
    };

    class LPOrderScheduler : public OrderScheduler
    {
    public:
        static VectorDynamic schedule(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, std::vector<uint> &processorJobVec)
        {            
            ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer(dagTasks);
            auto stv = SFOrderScheduling(dagTasks.tasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVec);
            scheduleOptimizer.setObjType(scheduleOptions.considerSensorFusion_);
            scheduleOptimizer.Optimize(stv, processorJobVec);
            return scheduleOptimizer.getOptimizedStartTimeVector();
        }
    };
} // namespace OrderOptDAG_SPACE