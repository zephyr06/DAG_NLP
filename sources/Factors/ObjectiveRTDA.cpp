#include "sources/Factors/ObjectiveRTDA.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        const std::string ObjectiveFunctionBase::type_trait("ObjectiveFunctionBase");
        const std::string RTDAExperimentObj::type_trait("RTDAExperimentObj");
        const std::string ReactionTimeObj::type_trait("ReactionTimeObj");
        const std::string DataAgeObj::type_trait("DataAgeObj");

        std::vector<std::vector<RTDA>> GetRTDAFromAllChains(
            const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
            const VectorDynamic &startTimeVector)
        {
            std::vector<std::vector<RTDA>> rtdaVec;
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
            {
                auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i],
                                                        startTimeVector);
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
        double GetOverallRt(const std::vector<std::vector<RTDA>> &rtdaVec)
        {
            double overall = 0;
            for (uint i = 0; i < rtdaVec.size(); i++)
                overall += ObjRT(rtdaVec[i]);
            return overall;
        }
        double GetOverallDa(const std::vector<std::vector<RTDA>> &rtdaVec)
        {
            double overall = 0;
            for (uint i = 0; i < rtdaVec.size(); i++)
            {
                overall += ObjDA(rtdaVec[i]);
            }
            return overall;
        }

        double RTDAExperimentObj::TrueObj(const DAG_Model &dagTasks,
                                          const TaskSetInfoDerived &tasksInfo,
                                          const VectorDynamic &startTimeVector,
                                          const ScheduleOptions scheduleOptions)
        {
            // BeginTimer("RTDAExperimentObj_TrueObj");
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);

            double res = ObjRTDA(maxRtda);
            // EndTimer("RTDAExperimentObj_TrueObj");
            return res;
        }

        double RTDAExperimentObj::Evaluate(const DAG_Model &dagTasks,
                                           const TaskSetInfoDerived &tasksInfo,
                                           const VectorDynamic &startTimeVector,
                                           const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallRTDA = GetOverallRtda(rtdaVec);

            double res =
                overallRTDA * scheduleOptions.weightInMpRTDA_ + ObjRTDA(maxRtda);
            return res;
        }

        double ReactionTimeObj::TrueObj(const DAG_Model &dagTasks,
                                        const TaskSetInfoDerived &tasksInfo,
                                        const VectorDynamic &startTimeVector,
                                        const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);

            double res = ObjRT(maxRtda);
            return res;
        }

        double ReactionTimeObj::Evaluate(const DAG_Model &dagTasks,
                                         const TaskSetInfoDerived &tasksInfo,
                                         const VectorDynamic &startTimeVector,
                                         const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallRt = GetOverallRt(rtdaVec);
            double res =
                overallRt * scheduleOptions.weightInMpRTDA_ + ObjRT(maxRtda);
            return res;
        }
        double DataAgeObj::TrueObj(const DAG_Model &dagTasks,
                                   const TaskSetInfoDerived &tasksInfo,
                                   const VectorDynamic &startTimeVector,
                                   const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);

            double res = ObjDA(maxRtda);
            return res;
        }

        double DataAgeObj::Evaluate(const DAG_Model &dagTasks,
                                    const TaskSetInfoDerived &tasksInfo,
                                    const VectorDynamic &startTimeVector,
                                    const ScheduleOptions scheduleOptions)
        {
            std::vector<std::vector<RTDA>> rtdaVec =
                GetRTDAFromAllChains(dagTasks, tasksInfo, startTimeVector);
            std::vector<RTDA> maxRtda = GetMaxRTDAs(rtdaVec);
            double overallDa = GetOverallDa(rtdaVec);
            double res =
                overallDa * scheduleOptions.weightInMpRTDA_ + ObjDA(maxRtda);
            return res;
        }

    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE