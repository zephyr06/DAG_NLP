#pragma once
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
// #include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Optimization/SFOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/Interval.h"
#include "sources/Factors/SensorFusionFactor.h"

namespace OrderOptDAG_SPACE
{

    // TODO: clear this struct
    struct ScheduleResult
    {
        // JobOrder jobOrder_;
        OrderOptDAG_SPACE::SFOrder sfOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        // RTDA rtda_;
        double obj_;
        double objWeighted_;
        double timeTaken_;
        std::vector<uint> processorJobVec_;
        LLint countOutermostWhileLoop_;
        LLint countMakeProgress_;
        LLint countIterationStatus_;

        ScheduleResult() { obj_ = -1; }
        // ScheduleResult(JobOrder jobOrder,
        //                VectorDynamic startTimeVector,
        //                bool schedulable,
        //                double obj) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), obj_(obj) //, rtda_(rtda)
        // {
        //     timeTaken_ = 0;
        //     countOutermostWhileLoop_ = 0;
        //     countMakeProgress_ = 0;
        //     countIterationStatus_ = 0;
        // }
        // ScheduleResult(JobOrder jobOrder, VectorDynamic startTimeVector, bool schedulable,
        //                double obj, std::vector<uint> processorJobVec)
        //     : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable),
        //       obj_(obj), processorJobVec_(processorJobVec)
        // {
        //     timeTaken_ = 0;
        //     countOutermostWhileLoop_ = 0;
        //     countMakeProgress_ = 0;
        //     countIterationStatus_ = 0;
        // }
        ScheduleResult(SFOrder sfOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       double obj) : sfOrder_(sfOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), obj_(obj) //, rtda_(rtda)
        {
            timeTaken_ = 0;
            countOutermostWhileLoop_ = 0;
            countMakeProgress_ = 0;
            countIterationStatus_ = 0;
        }
        ScheduleResult(SFOrder sfOrder, VectorDynamic startTimeVector, bool schedulable,
                       double obj, std::vector<uint> processorJobVec)
            : sfOrder_(sfOrder), startTimeVector_(startTimeVector), schedulable_(schedulable),
              obj_(obj), processorJobVec_(processorJobVec)
        {
            timeTaken_ = 0;
            countOutermostWhileLoop_ = 0;
            countMakeProgress_ = 0;
            countIterationStatus_ = 0;
        }

        void print()
        {
            std::cout << "Objective is: " << obj_ << std::endl;
        }
    };

    void PrintResultAnalyzation(ScheduleResult &scheduleResult, DAG_Model &dagTasks);

    bool CheckDDLConstraint(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector);

    std::vector<std::vector<Interval>> ExtractJobsPerProcessor(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum);

    double DBF_Error(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum);

    // Exams DBF constraints
    bool ExamDBF_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum);

    inline bool ExamDDL_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        return CheckDDLConstraint(dagTasks, tasksInfo, startTimeVector);
    }

    bool ExamBasic_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum);

    bool ExamAll_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
                             VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec,
                             int processorNum, double sfBound, double freshnessBound);

    ScheduleResult ScheduleDAGLS_LFT(DAG_Model &dagTasks, int processorNum, double sfBound, double freshnessBound);
} // namespace OrderOptDAG_SPACE