#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/SensorFusionFactor.h"

namespace OrderOptDAG_SPACE
{

    struct ScheduleResult
    {
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        RTDA rtda_;
        double obj_;
        double timeTaken_;
        std::vector<uint> processorJobVec_;

        ScheduleResult() {}
        ScheduleResult(JobOrder jobOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       RTDA rtda) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), rtda_(rtda)
        {
            obj_ = ObjRTDA(rtda_);
            timeTaken_ = 0;
            processorJobVec_.clear();
        }
    };

    void PrintResultAnalyzation(ScheduleResult &scheduleResult, DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::cout << Color::blue;
        std::cout << "Schedulable after optimization? " << scheduleResult.schedulable_ << std::endl;
        RTDA resAfterOpt = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], scheduleResult.startTimeVector_);
        resAfterOpt.print();
        std::cout << Color::def << std::endl;
        std::cout << "Schedule: " << std::endl;
        PrintSchedule(tasksInfo, scheduleResult.startTimeVector_);
    }

    bool CheckDDLConstraint(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        gtsam::NonlinearFactorGraph graph;
        AddDDL_Factor(graph, tasksInfo);
        gtsam::Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        double err = graph.error(initialEstimateFG);
        return 0 == err;
    }

    // Exams DBF constraints
    bool ExamDBF_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {
        if (processorNum <= 0)
            return false;
        std::vector<std::vector<Interval>> jobsPerProcessor(processorNum);
        int index = 0;
        for (uint i = 0; i < dagTasks.tasks.size(); i++)
        {
            for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC job(i, j);
                Interval v(GetStartTime(job, startTimeVector, tasksInfo), tasksInfo.tasks[i].executionTime);
                if (v.start < tasksInfo.tasks[i].period * j)
                    return false;
                else if (v.start + v.length > tasksInfo.tasks[i].period * j + tasksInfo.tasks[i].deadline)
                    return false;
                if (processorJobVec[index] >= jobsPerProcessor.size())
                    return false;
                jobsPerProcessor[processorJobVec[index]].push_back(v);
                index++;
            }
        }
        for (int i = 0; i < processorNum; i++)
        {
            if (IntervalOverlapError(jobsPerProcessor[i]) > 0)
                return false;
        }
        return true;
    }

    bool ExamDDL_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        return CheckDDLConstraint(dagTasks, tasksInfo, startTimeVector);
    }

    bool ExamAll_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {
        if (!ExamDDL_Feasibility(dagTasks, tasksInfo, startTimeVector) || !ExamDBF_Feasibility(dagTasks, tasksInfo, startTimeVector, processorJobVec, processorNum))
            return false;
        return true;
    }

    bool ExamAll_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum, double sfBound, double freshnessBound)
    {
        if (!ExamDDL_Feasibility(dagTasks, tasksInfo, startTimeVector) || !ExamDBF_Feasibility(dagTasks, tasksInfo, startTimeVector, processorJobVec, processorNum))
            return false;

        if (considerSensorFusion)
        {
            // Exam RTDA
            std::vector<OrderOptDAG_SPACE::RTDA> rtdaVec = OrderOptDAG_SPACE::GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[0], startTimeVector);
            OrderOptDAG_SPACE::RTDA rtda = GetMaxRTDA(rtdaVec);
            if (rtda.dataAge > freshnessBound || rtda.reactionTime > freshnessBound)
                return false;

            // Exam SF
            VectorDynamic sfError = OrderOptDAG_SPACE::ObtainSensorFusionError(dagTasks, tasksInfo, startTimeVector);
            if (sfError.maxCoeff() > sfBound)
                return false;
        }
        return true;
    }

    ScheduleResult ScheduleDAGLS_LFT(DAG_Model &dagTasks, int processorNum, double sfBound, double freshnessBound)
    {
        TaskSet &tasks = dagTasks.tasks;
        RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
        std::vector<uint> processorJobVec;
        VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum, std::nullopt, processorJobVec);
        JobOrder jobOrderRef(tasksInfo, initialSTV);
        RTDA rtda = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], initialSTV);
        ScheduleResult res{
            jobOrderRef,
            initialSTV,
            true, rtda};
        res.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec, processorNum, sfBound, freshnessBound);

        return res;
    }
} // namespace OrderOptDAG_SPACE