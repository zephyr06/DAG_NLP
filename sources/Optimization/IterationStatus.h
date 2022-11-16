#pragma once

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

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        struct ScheduleOptions
        {
            int causeEffectChainNumber_;
            bool considerSensorFusion_;
            bool doScheduleOptimization_;
            bool doScheduleOptimizationOnlyOnce_;
            int processorNum_;

            // some weights used in objective function evaluation
            double freshTol_;
            double sensorFusionTolerance_;

            double weightInMpRTDA_;
            double weightInMpSf_;
            double weightPunish_;

            ScheduleOptions() : causeEffectChainNumber_(1), considerSensorFusion_(0), doScheduleOptimization_(0), doScheduleOptimizationOnlyOnce_(0), processorNum_(2), freshTol_(100), sensorFusionTolerance_(100),
                                weightInMpRTDA_(0.5), weightInMpSf_(0.5), weightPunish_(10) {}

            void LoadParametersYaml()
            {
                causeEffectChainNumber_ = NumCauseEffectChain;
                considerSensorFusion_ = considerSensorFusion;
                doScheduleOptimization_ = doScheduleOptimization;
                doScheduleOptimizationOnlyOnce_ = doScheduleOptimizationOnlyOnce;
                processorNum_ = coreNumberAva;

                freshTol_ = freshTol;
                sensorFusionTolerance_ = sensorFusionTolerance;
                weightInMpRTDA_ = weightInMpRTDA;
                weightInMpSf_ = weightInMpSf;
                weightPunish_ = weightInMpRTDAPunish;
            }
        };
        static int infeasibleCount = 0;
        extern int infeasibleCount;
        template <typename OrderScheduler>
        struct IterationStatus
        {
            DAG_Model dagTasks_;
            SFOrder &jobOrder_;
            VectorDynamic startTimeVector_;
            std::vector<uint> processorJobVec_;
            std::vector<std::vector<RTDA>> rtdaVec_; // for each chain
            std::vector<RTDA> maxRtda_;              // for each chain
            // double objVal_;
            bool schedulable_; // only basic schedulability
            VectorDynamic sfVec_;
            double objWeighted_;
            ScheduleOptions scheduleOptions_;

            IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, const ScheduleOptions &schedultOptions) : dagTasks_(dagTasks), jobOrder_(jobOrder), scheduleOptions_(schedultOptions)
            {
                // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
                BeginTimerAppInProfiler;
                processorJobVec_.clear();
                startTimeVector_ = SFOrderScheduling(dagTasks, tasksInfo, scheduleOptions_.processorNum_, jobOrder_, processorJobVec_);
                schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, scheduleOptions_.processorNum_);
                if (!schedulable_)
                {
                    RTDA temp(1e9, 1e9);
                    for (uint i = 0; i < dagTasks.chains_.size(); i++)
                    {
                        maxRtda_.push_back(temp);
                        rtdaVec_.push_back(maxRtda_);
                    }
                    if (scheduleOptions_.considerSensorFusion_)
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
                    if (scheduleOptions_.considerSensorFusion_)
                    {
                        sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                    }
                }
                objWeighted_ = ObjWeighted();
                // TODO(Dong): this part of code does too many things, and is confusing for Sen to read and understand
                if (schedulable_ && scheduleOptions_.doScheduleOptimization_ && !scheduleOptions_.doScheduleOptimizationOnlyOnce_)
                {
                    BeginTimer("LP_Iterations");
                    ScheduleResult scheduleResBeforeOpt{jobOrder_, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};
                    scheduleResBeforeOpt.objWeighted_ = objWeighted_;

                    ScheduleResult resultAfterOptimization;
                    ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
                    BeginTimer("LP_Optimization");
                    scheduleOptimizer.OptimizeObjWeighted(dagTasks, scheduleResBeforeOpt);
                    EndTimer("LP_Optimization");
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
                        schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, scheduleOptions_.processorNum_);
                        jobOrder_ = SFOrder(tasksInfo, startTimeVector_); // jobOrder_ will be accessed later
                        objWeighted_ = ObjWeighted();
                    }
                    EndTimer("LP_Iterations");
                }
                EndTimerAppInProfiler;
            }

            IterationStatus &operator=(IterationStatus &status)
            {
                dagTasks_ = status.dagTasks_;
                jobOrder_ = status.jobOrder_;
                scheduleOptions_ = status.scheduleOptions_;
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
                double res = overallRTDA * scheduleOptions_.weightInMpRTDA_;
                if (scheduleOptions_.considerSensorFusion_ == 0)
                {
                    // Optmize RTDA: obj = max_RTs + max_DAs + overallRTDA * weightRTDA
                    res += ReadObj();
                }
                else
                {
                    // only used in RTSS21IC experiment
                    // Optimize Sensor Fusion: obj = overallRTDA * someWeight + overallSensorFusion * someWeight +
                    // {Barrier(max(RT)) * w_punish + Barrier(max(DA)) * w_punish}**for_every_chain + Barrier(max(SensorFusion)) * w_punish
                    double sfOverall = sfVec_.sum();
                    res += sfOverall * scheduleOptions_.weightInMpSf_;
                    res += ObjBarrier();
                }
                return res;
            }
            double ObjBarrier()
            {
                if (scheduleOptions_.considerSensorFusion_ == 0)
                    return ReadObj();
                else
                {
                    double error = 0;
                    if (sfVec_.rows() > 0)
                        error = Barrier(scheduleOptions_.sensorFusionTolerance_ - sfVec_.maxCoeff()) * scheduleOptions_.weightPunish_;
                    for (uint i = 0; i < dagTasks_.chains_.size(); i++)
                    {
                        error += Barrier(scheduleOptions_.freshTol_ - maxRtda_[i].reactionTime) * scheduleOptions_.weightPunish_ + Barrier(scheduleOptions_.freshTol_ - maxRtda_[i].dataAge) * scheduleOptions_.weightPunish_;
                    }
                    return error;
                }
            }
        };

        template <typename OrderScheduler>
        bool MakeProgress(IterationStatus<OrderScheduler> &statusPrev, IterationStatus<OrderScheduler> &statusCurr)
        {
            if (!statusCurr.schedulable_)
            {
                infeasibleCount++;
                if (debugMode == 1)
                {
                    TaskSetInfoDerived tasksInfo(statusCurr.dagTasks_.tasks);
                    std::cout << "Infeasible schedule #:" << infeasibleCount << std::endl;
                    // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                    statusCurr.jobOrder_.print();
                }
                return false;
            }
            if (statusCurr.objWeighted_ < statusPrev.objWeighted_)
                return true;
            return false;
        }
    }
}