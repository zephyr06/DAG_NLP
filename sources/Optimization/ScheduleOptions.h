#pragma once
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        struct ScheduleOptions
        {
            int causeEffectChainNumber_;
            bool considerSensorFusion_; // TODO: remove this parameter
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
    }
}