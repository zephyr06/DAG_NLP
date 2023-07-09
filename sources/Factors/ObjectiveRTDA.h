
#pragma once
#include "sources/Factors/RTDA_Analyze.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/testMy.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        std::vector<std::vector<RTDA>> GetRTDAFromAllChains(const DAG_Model &dagTasks,
                                                            const TaskSetInfoDerived &tasksInfo,
                                                            const VectorDynamic &startTimeVector);

        std::vector<RTDA> GetMaxRTDAs(const std::vector<std::vector<RTDA>> &rtdaVec);

        double GetOverallRtda(const std::vector<std::vector<RTDA>> &rtdaVec);
        double GetOverallRt(const std::vector<std::vector<RTDA>> &rtdaVec);
        double GetOverallDa(const std::vector<std::vector<RTDA>> &rtdaVec);

        class ObjectiveFunctionBase
        {
        public:
            static const std::string type_trait;

            static double TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);

            static double Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
            {
                CoutError("Base function should not be called!");
                return 0;
            }
        };

        class RTDAExperimentObj : ObjectiveFunctionBase
        {
        public:
            static const std::string type_trait;
            static double TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);

            static double Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);
        };

        class ReactionTimeObj : ObjectiveFunctionBase
        {
        public:
            static const std::string type_trait;
            static double TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);

            static double Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);
        };

        class DataAgeObj : ObjectiveFunctionBase
        {
        public:
            static const std::string type_trait;
            static double TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                  const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);

            static double Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions);
        };

    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE