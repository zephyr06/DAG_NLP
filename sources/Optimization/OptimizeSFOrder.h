#pragma once
#include <algorithm> // for copy() and assign()
#include <iterator>  // for back_inserter

#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Optimization/IterationStatus.h"
#include "sources/Optimization/SkipUnschedulablePermutations.h"
#include "sources/Optimization/OrderScheduler.h"
// #include "sources/Tools/profilier.h"

namespace OrderOptDAG_SPACE
{
    // template <typename OrderScheduler>
    namespace OptimizeSF
    {

        std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks);

        template <typename OrderScheduler>
        ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, const ScheduleOptions &schedultOptions, boost::optional<ScheduleResult &> resOrderOptWithoutScheduleOpt = boost::none);
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
