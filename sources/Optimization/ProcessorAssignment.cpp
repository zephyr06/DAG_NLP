#include "sources/Optimization/ProcessorAssignment.h"

#include "sources/Optimization/SFOrder.h"
#include "sources/Utils/DeclareDAG.h"

namespace OrderOptDAG_SPACE {
// this function usually is fast
bool ProcessorAssignment::AssignProcessor(const TaskSetInfoDerived &tasksInfo, const SFOrder &sfOrder,
                                          const int processorNum, std::vector<uint> &processorJobVec) {
  processorJobVec.clear();
  processorJobVec.resize(tasksInfo.variableDimension, 0);

  std::vector<bool> processorBusy(processorNum, false);
  bool flagProcessorAvailable;

  for (LLint i = 0; i < sfOrder.size(); i++) {
    const auto &orderInstance = sfOrder.at(i);
    if (orderInstance.type == 's') {
      flagProcessorAvailable = false;
      for (int n = 0; n < processorNum; n++) {
        if (!processorBusy[n]) {
          flagProcessorAvailable = true;
          processorBusy[n] = true;
          processorJobVec[IndexTran_Instance2Overall(orderInstance.job.taskId, orderInstance.job.jobId,
                                                     tasksInfo.sizeOfVariables)] = n;
          break;
        }
      }
      if (!flagProcessorAvailable) {
        processorJobVec.clear();
        return false; // SFOrder unschedulable
      }
    } else // finish instance
    {
      processorBusy[processorJobVec[IndexTran_Instance2Overall(
          orderInstance.job.taskId, orderInstance.job.jobId, tasksInfo.sizeOfVariables)]] = false;
    }
  }
  return true;
}

} // namespace OrderOptDAG_SPACE