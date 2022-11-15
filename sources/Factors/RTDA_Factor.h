#pragma once

#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Utils/JobCEC.h"

namespace OrderOptDAG_SPACE
{
    struct RTDA
    {
        double reactionTime;
        double dataAge;
        RTDA() : reactionTime(-1), dataAge(-1) {}
        RTDA(double r, double d) : reactionTime(r), dataAge(d) {}
        void print()
        {
            std::cout << "Reaction time is " << reactionTime << ", data age is " << dataAge << std::endl;
        }
    };

    RTDA GetMaxRTDA(std::vector<RTDA> &resVec);

    std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const VectorDynamic &x);

    std::vector<RTDA> GetRTDAFromSingleJob(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const Values &x);

    RTDA GetMaxRTDA(const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, VectorDynamic &startTimeVector);

    double ObjRTDA(const RTDA &rtda);
    double ObjRTDA(const std::vector<RTDA> &rtdaVec);

    std::unordered_map<JobCEC, std::vector<JobCEC>> GetRTDAReactChainsFromSingleJob(
        const TaskSetInfoDerived &tasksInfo, const std::vector<int> &causeEffectChain, const VectorDynamic &x);

}