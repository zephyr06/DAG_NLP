#pragma once
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{

    struct ScheduleResult
    {
        JobOrder jobOrder_;
        SFOrder sfOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        // RTDA rtda_;
        double obj_;
        double timeTaken_;
        std::vector<uint> processorJobVec_;

        ScheduleResult() { obj_ = -1; }
        ScheduleResult(JobOrder jobOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       double obj) : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), obj_(obj) //, rtda_(rtda)
        {
            timeTaken_ = 0;
        }
        ScheduleResult(JobOrder jobOrder, VectorDynamic startTimeVector, bool schedulable,
                       double obj, std::vector<uint> processorJobVec)
            : jobOrder_(jobOrder), startTimeVector_(startTimeVector), schedulable_(schedulable),
              obj_(obj), processorJobVec_(processorJobVec)
        {
            timeTaken_ = 0;
        }
        ScheduleResult(SFOrder sfOrder,
                       VectorDynamic startTimeVector,
                       bool schedulable,
                       double obj) : sfOrder_(sfOrder), startTimeVector_(startTimeVector), schedulable_(schedulable), obj_(obj) //, rtda_(rtda)
        {
            timeTaken_ = 0;
        }
        ScheduleResult(SFOrder sfOrder, VectorDynamic startTimeVector, bool schedulable,
                       double obj, std::vector<uint> processorJobVec)
            : sfOrder_(sfOrder), startTimeVector_(startTimeVector), schedulable_(schedulable),
              obj_(obj), processorJobVec_(processorJobVec)
        {
            timeTaken_ = 0;
        }
        
        void print()
        {
            std::cout << "Objective is: " << obj_ << std::endl;
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

    std::vector<std::vector<Interval>> ExtractJobsPerProcessor(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {

        std::vector<std::vector<Interval>> jobsPerProcessor(processorNum);
        if (processorNum <= 0)
            return jobsPerProcessor;
        int index = 0;
        for (uint i = 0; i < dagTasks.tasks.size(); i++)
        {
            for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
            {
                JobCEC job(i, j);
                Interval v(GetStartTime(job, startTimeVector, tasksInfo), tasksInfo.tasks[i].executionTime);
                if (processorJobVec[index] >= jobsPerProcessor.size())
                {
                    CoutWarning("Wrong processorNum in ExtractJobsPerProcessor!");
                    // jobsPerProcessor.resize(processorJobVec[index] + 1);
                    while (jobsPerProcessor.size() < processorJobVec[index] + 1)
                    {
                        std::vector<Interval> ttt;
                        jobsPerProcessor.push_back(ttt);
                    }
                    jobsPerProcessor[processorJobVec[index]].push_back(v);
                }
                else
                    jobsPerProcessor[processorJobVec[index]].push_back(v);
                index++;
            }
        }
        return jobsPerProcessor;
    }

    double DBF_Error(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {
        if (processorNum <= 0)
            return 0;
        if (processorJobVec.size() == 0)
            CoutError("Empty processorJobVecin DBF_Error!");
        std::vector<std::vector<Interval>> jobsPerProcessor = ExtractJobsPerProcessor(dagTasks, tasksInfo, startTimeVector, processorJobVec, processorNum);
        double overallError = 0;
        for (int i = 0; i < processorNum; i++)
        {
            overallError += IntervalOverlapError(jobsPerProcessor[i]);
        }
        return overallError;
    }

    // Exams DBF constraints
    bool ExamDBF_Feasibility(DAG_Model &dagTasks, RegularTaskSystem::TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector, std::vector<uint> &processorJobVec, int processorNum)
    {
        if (processorNum <= 0)
            return false;
        std::vector<std::vector<Interval>> jobsPerProcessor = ExtractJobsPerProcessor(dagTasks, tasksInfo, startTimeVector, processorJobVec, processorNum);
        if (static_cast<int>(jobsPerProcessor.size()) > processorNum)
            return false;
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
        std::vector<RTDA> maxRtdaVec;
        // double obj = 0;
        for (uint i = 0; i < dagTasks.chains_.size(); i++)
        {
            RTDA rtda = GetMaxRTDA(tasksInfo, dagTasks.chains_[i], initialSTV);
            maxRtdaVec.push_back(rtda);
        }

        ScheduleResult res{
            jobOrderRef,
            initialSTV,
            true, ObjRTDA(maxRtdaVec)};
        res.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, initialSTV, processorJobVec, processorNum, sfBound, freshnessBound);

        return res;
    }
} // namespace OrderOptDAG_SPACE