#pragma once
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE {
namespace OptimizeSF {
struct ScheduleOptions {
    int causeEffectChainNumber_;
    int processorNum_;

    // some weights used in objective function evaluation
    double freshTol_;
    double sensorFusionTolerance_;

    int selectInitialFromPoolCandidate_;
    double weightInMpRTDA_;
    double weightInMpSf_;
    double weightPunish_;

    ScheduleOptions()
        : causeEffectChainNumber_(1),
          processorNum_(2),
          freshTol_(100),
          sensorFusionTolerance_(100),
          selectInitialFromPoolCandidate_(0),
          weightInMpRTDA_(0.5),
          weightInMpSf_(0.5),
          weightPunish_(10) {}

    void LoadParametersYaml() {
        causeEffectChainNumber_ = GlobalVariablesDAGOpt::NumCauseEffectChain;
        processorNum_ = GlobalVariablesDAGOpt::coreNumberAva;

        freshTol_ = GlobalVariablesDAGOpt::freshTol;
        sensorFusionTolerance_ = GlobalVariablesDAGOpt::sensorFusionTolerance;
        selectInitialFromPoolCandidate_ =
            GlobalVariablesDAGOpt::selectInitialFromPool;
        weightInMpRTDA_ = GlobalVariablesDAGOpt::weightInMpRTDA;
        weightInMpSf_ = GlobalVariablesDAGOpt::weightInMpSf;
        weightPunish_ = GlobalVariablesDAGOpt::weightInMpRTDAPunish;
    }
};

}  // namespace OptimizeSF

}  // namespace OrderOptDAG_SPACE