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
// #include "sources/Optimization/InitialEstimate.h"

namespace OrderOptDAG_SPACE
{
    class OrderScheduler
    {
    public:
        // jobOrder is allowed to change
        OrderScheduler() {}
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, const std::vector<uint> processorJobVe)
        {
            CoutError("Never call base function!");
            return GenerateVectorDynamic1D(0);
        }
    };

    class SimpleOrderScheduler : public OrderScheduler
    {
    public:
        // processorJobVe will be assigned values
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, std::vector<uint> &processorJobVe)
        {
            return SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVe);
        }
    };

    // TODO(Dong): make this function work
    class LPOrderScheduler : public OrderScheduler
    {
    public:
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const OptimizeSF::ScheduleOptions &scheduleOptions, SFOrder &jobOrder, std::vector<uint> &processorJobVe)
        {
            // BeginTimer("LPOrderScheduler");
            // ScheduleResult scheduleResBeforeOpt{jobOrder, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};

            // ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
            // scheduleOptimizer.OptimizeObjWeighted(dagTasks, scheduleResBeforeOpt);
            // ScheduleResult resultAfterOptimization = scheduleOptimizer.getOptimizedResult();

            // EndTimer("LPOrderScheduler");
            // return resultAfterOptimization.startTimeVector_;

            return SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions.processorNum_, jobOrder, processorJobVe);
        }
    };
} // namespace OrderOptDAG_SPACE