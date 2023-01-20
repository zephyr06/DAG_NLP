#pragma once
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{
    class ProcessorAssignment
    {
    public:
        ProcessorAssignment() = default;
        
        /*
        Return false if SFOrder disobey processor number constraints(necessary of unschedulalble), otherwise return true.
        Assigned processor will be written into the input vector processorJobVec.
        */
        static bool AssignProcessor(const TaskSetInfoDerived &tasksInfo,const SFOrder &sfOrder,
                                    const int processorNum, std::vector<uint> &processorJobVec);
    };

} // namespace OrderOptDAG_SPACE