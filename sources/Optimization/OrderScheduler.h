#pragma once
#include "unordered_set"
#include "unordered_map"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
// #include "sources/Optimization/InitialEstimate.h"

namespace OrderOptDAG_SPACE
{
    class OrderScheduler
    {
        // startTimeVector_ = SFOrderScheduling(dagTasks, tasksInfo, processorNum_, jobOrder_, processorJobVec_);
        OrderScheduler() {}
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, int processorNum, const SFOrder &jobOrder, const std::vector<uint> processorJobVe)
        {
            CoutError("Never call base function!");
            return GenerateVectorDynamic1D(0);
        }
    };

    class SimpleOrderScheduler : public OrderScheduler
    {
        // processorJobVe will be assigned values
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, int processorNum, const SFOrder &jobOrder, std::vector<uint> &processorJobVe)
        {
            return SFOrderScheduling(dagTasks, tasksInfo, processorNum, jobOrder, processorJobVe);
        }
    };

    // TODO(Dong): make this function work
    class LPOrderScheduler : public OrderScheduler
    {
        static VectorDynamic schedule(DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, int processorNum, const SFOrder &jobOrder, std::vector<uint> &processorJobVe)
        {
            // BeginTimer("LPOrderScheduler");
            // ScheduleResult scheduleResBeforeOpt{jobOrder, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};

            // ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
            // scheduleOptimizer.OptimizeObjWeighted(dagTasks, scheduleResBeforeOpt);
            // ScheduleResult resultAfterOptimization = scheduleOptimizer.getOptimizedResult();

            // EndTimer("LPOrderScheduler");
            // return resultAfterOptimization.startTimeVector_;

            return SFOrderScheduling(dagTasks, tasksInfo, processorNum, jobOrder, processorJobVe);
        }
    };
} // namespace OrderOptDAG_SPACE