#pragma once
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{
    class ProcessorAssignment
    {
    public:
        ProcessorAssignment() = default;
        static bool AssignProcessor(const TaskSetInfoDerived &tasksInfo, SFOrder &sfOrder,
                                    const int processorNum, std::vector<uint> &processorJobVec);
    };

} // namespace OrderOptDAG_SPACE