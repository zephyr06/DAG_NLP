#include "sources/Optimization/ObjectiveFunctions.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        const std::string ObjectiveFunctionBase::type_trait("ObjectiveFunctionBase");
        const std::string RTDAExperimentObj::type_trait("RTDAExperimentObj");
        const std::string RTSS21ICObj::type_trait("RTSS21ICObj");

        std::vector<std::vector<RTDA>> GetRTDAFromAllChains(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
        {
            std::vector<std::vector<RTDA>> rtdaVec;
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
            {
                auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector);
                rtdaVec.push_back(rtdaVecTemp);
            }
            return rtdaVec;
        }

        std::vector<RTDA> GetMaxRTDAs(const std::vector<std::vector<RTDA>> &rtdaVec)
        {
            std::vector<RTDA> maxRtda;
            for (uint i = 0; i < rtdaVec.size(); i++)
                maxRtda.push_back(GetMaxRTDA(rtdaVec.at(i)));

            return maxRtda;
        }

        double GetOverallRtda(const std::vector<std::vector<RTDA>> &rtdaVec)
        {
            double overallRTDA = 0;
            for (uint i = 0; i < rtdaVec.size(); i++)
                overallRTDA += ObjRTDA(rtdaVec[i]);
            return overallRTDA;
        }

        double RTDAExperimentObj::TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec = GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);

            double res = ObjRTDA(maxRtda);
            return res;
        }

        double RTDAExperimentObj::Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec = GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallRTDA = GetOverallRtda(rtdaVec);

            double res = overallRTDA * scheduleOptions.weightInMpRTDA_ + ObjRTDA(maxRtda);
            return res;
        }

        double RTSS21ICObj::EvaluateRTDA(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {

            std::vector<std::vector<RTDA>> rtdaVec = GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallRTDA = GetOverallRtda(rtdaVec);

            double resFromRTDA = overallRTDA * scheduleOptions.weightInMpRTDA_;
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
            {
                resFromRTDA += Barrier(scheduleOptions.freshTol_ - maxRtda[i].reactionTime) * scheduleOptions.weightPunish_ + Barrier(scheduleOptions.freshTol_ - maxRtda[i].dataAge) * scheduleOptions.weightPunish_;
            }
            return resFromRTDA;
        }

        double RTSS21ICObj::EvaluateSF(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {
            VectorDynamic sfVec = ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);

            double sfOverall = sfVec.sum();
            double resSF = sfOverall * scheduleOptions.weightInMpSf_;
            if (sfVec.rows() > 0)
                resSF += Barrier(scheduleOptions.sensorFusionTolerance_ - sfVec.maxCoeff()) * scheduleOptions.weightPunish_;
            return resSF;
        }

        double RTSS21ICObj::Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {
            return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) + EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
        }

        double RTSS21ICObj::TrueObj(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
        {
            return EvaluateRTDA(dagTasks, tasksInfo, startTimeVector, scheduleOptions) + EvaluateSF(dagTasks, tasksInfo, startTimeVector, scheduleOptions);
        }

    }
}