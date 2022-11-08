#pragma once

#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        struct IterationStatus
        {
            DAG_Model dagTasks_;
            SFOrder &jobOrder_;
            int processorNum_;
            VectorDynamic startTimeVector_;
            std::vector<uint> processorJobVec_;
            std::vector<std::vector<RTDA>> rtdaVec_; // for each chain
            std::vector<RTDA> maxRtda_;              // for each chain
            // double objVal_;
            bool schedulable_; // only basic schedulability
            VectorDynamic sfVec_;
            double objWeighted_;

            IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, int processorNum) : dagTasks_(dagTasks), jobOrder_(jobOrder), processorNum_(processorNum)
            {
                // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
                BeginTimerAppInProfiler;
                processorJobVec_.clear();
                startTimeVector_ = SFOrderScheduling(dagTasks, tasksInfo, processorNum_, jobOrder_, processorJobVec_);
                schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_);
                if (!schedulable_)
                {
                    RTDA temp(1e9, 1e9);
                    for (uint i = 0; i < dagTasks.chains_.size(); i++)
                    {
                        maxRtda_.push_back(temp);
                        rtdaVec_.push_back(maxRtda_);
                    }
                    if (considerSensorFusion)
                        sfVec_ = GenerateVectorDynamic1D(1e9);
                }
                else
                {
                    for (uint i = 0; i < dagTasks.chains_.size(); i++)
                    {
                        auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                        rtdaVec_.push_back(rtdaVecTemp);
                        maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
                    }
                    if (considerSensorFusion)
                    {
                        sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                    }
                }
                objWeighted_ = ObjWeighted();
                if (schedulable_ && doScheduleOptimization && !doScheduleOptimizationOnlyOnce)
                {
                    ScheduleResult scheduleResBeforeOpt{jobOrder_, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};
                    scheduleResBeforeOpt.objWeighted_ = objWeighted_;

                    ScheduleResult resultAfterOptimization;
                    ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
                    scheduleOptimizer.OptimizeObjWeighted(dagTasks, scheduleResBeforeOpt);
                    resultAfterOptimization = scheduleOptimizer.getOptimizedResult();
                    if (resultAfterOptimization.objWeighted_ < scheduleResBeforeOpt.objWeighted_)
                    {
                        startTimeVector_ = resultAfterOptimization.startTimeVector_;
                        rtdaVec_.clear();
                        maxRtda_.clear();
                        for (uint i = 0; i < dagTasks.chains_.size(); i++)
                        {
                            auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                            rtdaVec_.push_back(rtdaVecTemp);
                            maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
                        }
                        if (considerSensorFusion)
                        {
                            sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                        }
                        schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_);
                        // TODO: remove this?
                        jobOrder_ = SFOrder(tasksInfo, startTimeVector_);
                        objWeighted_ = ObjWeighted();
                    }
                }
                EndTimerAppInProfiler;
            }

            IterationStatus &operator=(IterationStatus &status)
            {
                dagTasks_ = status.dagTasks_;
                jobOrder_ = status.jobOrder_;
                processorNum_ = status.processorNum_;
                startTimeVector_ = status.startTimeVector_;
                processorJobVec_ = status.processorJobVec_;
                rtdaVec_ = status.rtdaVec_;
                maxRtda_ = status.maxRtda_;
                schedulable_ = status.schedulable_;
                sfVec_ = status.sfVec_;
                objWeighted_ = status.objWeighted_;
                return *this;
            }

            double ReadObj()
            {
                double res = ObjRTDA(maxRtda_);
                return res;
            }
            double ObjWeighted()
            {
                double overallRTDA = 0;
                for (uint i = 0; i < rtdaVec_.size(); i++)
                    overallRTDA += ObjRTDA(rtdaVec_[i]);
                double res = overallRTDA * weightInMpRTDA;
                if (considerSensorFusion == 0)
                {
                    // Optmize RTDA: obj = max_RTs + max_DAs + overallRTDA * weightInMpRTDA
                    res += ReadObj();
                }
                else
                {
                    // only used in RTSS21IC experiment
                    // Optimize Sensor Fusion: obj = overallRTDA * someWeight + overallSensorFusion * someWeight +
                    // {Barrier(max(RT)) * w_punish + Barrier(max(DA)) * w_punish}**for_every_chain + Barrier(max(SensorFusion)) * w_punish
                    double sfOverall = sfVec_.sum();
                    res += sfOverall * weightInMpSf;
                    res += ObjBarrier();
                }
                return res;
            }
            double ObjBarrier()
            {
                if (considerSensorFusion == 0)
                    return ReadObj();
                else
                {
                    double error = 0;
                    if (sfVec_.rows() > 0)
                        error = Barrier(sensorFusionTolerance - sfVec_.maxCoeff()) * weightInMpSfPunish;
                    for (uint i = 0; i < dagTasks_.chains_.size(); i++)
                    {
                        error += Barrier(FreshTol - maxRtda_[i].reactionTime) * weightInMpRTDAPunish + Barrier(FreshTol - maxRtda_[i].dataAge) * weightInMpRTDAPunish;
                    }
                    return error;
                }
            }
        };
    }
}