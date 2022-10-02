#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"

namespace DAG_SPACE
{
    struct ScheduleResult
    {
        JobOrder jobOrder_;
        VectorDynamic startTimeVector_;
        bool schedulable_;
        RTDA rtda_;

        ScheduleResult() {}
    };

    void PrintResultAnalyzation(ScheduleResult &scheduleResult, DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        std::cout << Color::blue;
        std::cout << "Schedulable after optimization? " << scheduleResult.schedulable_ << std::endl;
        RTDA resAfterOpt = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], scheduleResult.startTimeVector_);
        resAfterOpt.print();
        std::cout << Color::def << std::endl;
    }

    bool SchedulabilityCheck(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector)
    {
        gtsam::NonlinearFactorGraph graph;
        AddDBF_Factor(graph, tasksInfo);
        AddDDL_Factor(graph, tasksInfo);
        gtsam::Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        return 0 == graph.error(initialEstimateFG);
    }

    ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
        JobOrder jobOrderRef(tasksInfo, initialSTV);

        RTDA rtda = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], initialSTV);
        double rtdaMin = ObjRTDA(tasksInfo, dagTasks.chains_[0], initialSTV);
        bool findNewUpdate = true;

        ScheduleResult scheduleRes{jobOrderRef, initialSTV, false, rtda};
        while (findNewUpdate)
        {
            findNewUpdate = false;
            for (LLint i = 0; i < static_cast<LLint>(jobOrderRef.size()); i++)
            {
                for (LLint j = 0; j < static_cast<LLint>(jobOrderRef.size()); j++)
                {
                    JobOrder jobOrderCurr = jobOrderRef;
                    jobOrderCurr.ChangeJobOrder(i, j);
                    VectorDynamic startTimeVector = ListSchedulingGivenOrder(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension, jobOrderCurr);
                    if (SchedulabilityCheck(dagTasks, tasksInfo, startTimeVector))
                    {
                        double rtdaCurr = ObjRTDA(tasksInfo, dagTasks.chains_[0], startTimeVector);
                        if (rtdaCurr < rtdaMin)
                        {
                            rtdaMin = rtdaCurr;
                            findNewUpdate = true;
                            scheduleRes.jobOrder_ = jobOrderCurr;
                            scheduleRes.startTimeVector_ = startTimeVector;
                            scheduleRes.schedulable_ = true;
                            scheduleRes.rtda_ = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], startTimeVector);
                            std::cout << "Make progress after one switch!" << std::endl;
                        }
                    }
                }
            }
        }
        return scheduleRes;
    }

    ScheduleResult ScheduleDAGLS_LFT(DAG_Model &dagTasks)
    {
        TaskSet &tasks = dagTasks.tasks;
        TaskSetInfoDerived tasksInfo(tasks);
        VectorDynamic initialSTV = ListSchedulingLFT(dagTasks, tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
        JobOrder jobOrderRef(tasksInfo, initialSTV);
        RTDA rtda = GetMaxRTDA(tasksInfo, dagTasks.chains_[0], initialSTV);

        if (SchedulabilityCheck(dagTasks, tasksInfo, initialSTV))
            return ScheduleResult{
                jobOrderRef,
                initialSTV,
                true, rtda};
        else
            return ScheduleResult{jobOrderRef, initialSTV, false, rtda};
    }
} // namespace DAG_SPACE
