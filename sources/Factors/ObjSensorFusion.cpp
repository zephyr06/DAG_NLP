#include "sources/Factors/ObjSensorFusion.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        // TODO: add "SensorFusionObj"
        const std::string RTSS21ICObj::type_trait("RTSS21ICObj");

        double RTSS21ICObj::EvaluateRTDA(const DAG_Model &dagTasks,
                                         const TaskSetInfoDerived &tasksInfo,
                                         const VectorDynamic &startTimeVector,
                                         const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallRTDA = GetOverallRtda(rtdaVec);

            double resFromRTDA = overallRTDA * scheduleOptions.weightInMpRTDA_;
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
            {
                resFromRTDA +=
                    Barrier(scheduleOptions.freshTol_ - maxRtda[i].reactionTime) *
                        scheduleOptions.weightPunish_ +
                    Barrier(scheduleOptions.freshTol_ - maxRtda[i].dataAge) *
                        scheduleOptions.weightPunish_;
            }
            return resFromRTDA;
        }

        double RTSS21ICObj::EvaluateSF(const DAG_Model &dagTasks,
                                       const TaskSetInfoDerived &tasksInfo,
                                       const VectorDynamic &startTimeVector,
                                       const ScheduleOptions scheduleOptions)
        {
            VectorDynamic sfVec =
                ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);

            double sfOverall = sfVec.sum();
            double resSF = sfOverall * scheduleOptions.weightInMpSf_;
            if (sfVec.rows() > 0)
                resSF +=
                    Barrier(scheduleOptions.sensorFusionTolerance_ - sfVec.maxCoeff()) *
                    scheduleOptions.weightPunish_;
            return resSF;
        }

        double RTSS21ICObj::Evaluate(const DAG_Model &dagTasks,
                                     const TaskSetInfoDerived &tasksInfo,
                                     const VectorDynamic &startTimeVector,
                                     const ScheduleOptions scheduleOptions)
        {
            return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) +
                   EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
        }

        double RTSS21ICObj::TrueObj(const DAG_Model &dagTasks,
                                    const TaskSetInfoDerived &tasksInfo,
                                    const VectorDynamic &startTimeVector,
                                    const ScheduleOptions scheduleOptions)
        {
            return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) +
                   EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
        }

    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE