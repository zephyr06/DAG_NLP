#pragma once
#include "sources/Optimization/InitialEstimate.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include <cstdio>
#include <fstream>
#include <iostream>
// #include "sources/Optimization/JobOrder.h"
#include "sources/Factors/Interval.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"
// #include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Optimization/OrderScheduler.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Utils/profilier.h"

// #define SAVE_ITERATION_STAT

namespace OrderOptDAG_SPACE
{
  namespace OptimizeSF
  {

    inline std::string GetStatsSeqFileName() { return GlobalVariablesDAGOpt::testDataSetName + "_Stat.txt"; }

    // extern int infeasibleCount;
    inline void AppendResultsToFile(double val, const std::string &file = GetStatsSeqFileName())
    {
      std::ofstream outfileWrite;
      outfileWrite.open(file, std::ios_base::app | std::ios_base::out);
      outfileWrite << val << std::endl;
      outfileWrite.close();
    }

    template <typename OrderScheduler, typename ObjectiveFunctionBase>
    class IterationStatus
    {
    public:
      bool schedulable_; // only basic schedulability
      double objWeighted_;
      // SFOrder jobOrder_;
      VectorDynamic startTimeVector_;

      IterationStatus() : schedulable_(false), objWeighted_(1e9), startTimeVector_(GenerateVectorDynamic1D(0)) {}

      IterationStatus(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                      const ScheduleOptions &scheduleOptions)
      {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif
        std::vector<uint> processorJobVec;
        startTimeVector_ =
            OrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder, processorJobVec, ObjectiveFunctionBase::type_trait);
        schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec,
                                             scheduleOptions.processorNum_);

        // std::vector<uint> processorJobVecSimple;
        // auto stvSimple =
        //     SimpleOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder,
        //     processorJobVecSimple);
        // bool schedulableSimple_ = ExamBasic_Feasibility(dagTasks, tasksInfo, stvSimple, processorJobVecSimple,
        //                                                 scheduleOptions.processorNum_);
        // if (schedulableSimple_) {
        //   startTimeVector_ = stvSimple;
        // }
        UpdateObjWeighted(dagTasks, tasksInfo, scheduleOptions);
#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
      }

      IterationStatus(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                      const ScheduleOptions &scheduleOptions, VectorDynamic startTimeVector,
                      std::vector<uint> processorJobVec, bool schedulable)
          : schedulable_(schedulable), startTimeVector_(startTimeVector)
      {
#ifdef PROFILE_CODE
        BeginTimer(__FUNCTION__);
#endif
        // std::vector<uint> processorJobVecSimple;
        // auto stvSimple =
        //     SimpleOrderScheduler::schedule(dagTasks, tasksInfo, scheduleOptions, jobOrder,
        //     processorJobVecSimple);
        // bool schedulableSimple_ = ExamBasic_Feasibility(dagTasks, tasksInfo, stvSimple, processorJobVecSimple,
        //                                                 scheduleOptions.processorNum_);
        // if (schedulableSimple_) {
        //   startTimeVector_ = stvSimple;
        // }

        UpdateObjWeighted(dagTasks, tasksInfo, scheduleOptions);
#ifdef PROFILE_CODE
        EndTimer(__FUNCTION__);
#endif
      }

      void UpdateObjWeighted(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                             const ScheduleOptions &scheduleOptions)
      {
        if (!schedulable_)
          objWeighted_ = 1e9;
        else
        {
          objWeighted_ = ObjectiveFunctionBase::Evaluate(dagTasks, tasksInfo, startTimeVector_, scheduleOptions);
#ifdef SAVE_ITERATION_STAT
          AppendResultsToFile(
              ObjectiveFunctionBase::TrueObj(dagTasks, tasksInfo, startTimeVector_, scheduleOptions));
#endif
        }
      }
    };

    template <typename OrderScheduler, typename ObjectiveFunctionBase>
    bool MakeProgress(const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusPrev,
                      const IterationStatus<OrderScheduler, ObjectiveFunctionBase> &statusCurr)
    {
      if (!statusCurr.schedulable_)
      {
        return false;
      }
      if (statusCurr.objWeighted_ < statusPrev.objWeighted_)
        return true;
      return false;
    }
  } // namespace OptimizeSF

} // namespace OrderOptDAG_SPACE