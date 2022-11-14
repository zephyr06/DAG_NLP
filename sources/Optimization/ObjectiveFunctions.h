#pragma once
#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Tools/testMy.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        // Used for IterationStatus only
        class ObjectiveFunctionBase
        {
        public:
            static double Evaluate(onst DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
            {
                CoutError("Base function should not be called!");
                return 0;
            }
        };

        class RTDAExperimentObj : ObjectiveFunctionBase
        {
        public:
            static double Evaluate(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector, const ScheduleOptions scheduleOptions)
            {
                std::vector<std::vector<RTDA>> rtdaVec;
                std::vector<RTDA> maxRtda;

                for (uint i = 0; i < dagTasks.chains_.size(); i++)
                {
                    auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                    rtdaVec.push_back(rtdaVecTemp);
                    maxRtda.push_back(GetMaxRTDA(rtdaVecTemp));
                }

                double overallRTDA = 0;
                for (uint i = 0; i < rtdaVec.size(); i++)
                    overallRTDA += ObjRTDA(rtdaVec_[i]);
                double res = overallRTDA * scheduleOptions.weightInMpRTDA_ + ObjRTDA(maxRtda_);
                return res;
            }
        };
    }
}