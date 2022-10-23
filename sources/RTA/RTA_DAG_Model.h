#pragma once

#include "tbb/task_scheduler_init.h"

#include "problem.hpp"
#include "io.hpp"
#include "global/space.hpp"
// #include "dagSched/tests.h" // to add np_schedulability_analysis
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/Tools/profilier.h"

using namespace OrderOptDAG_SPACE;

namespace rt_num_opt
{
    size_t rtaCallingTimes;
    size_t rtaControl;
    inline void ResetCallingTimes()
    {
        rtaCallingTimes = 0;
        rtaControl = 0;
    }
    inline void IncrementRTAControl()
    {
        rtaControl++;
    }
    inline void IncrementCallingTimes()
    {
        rtaCallingTimes++;
    }
    inline size_t ReadCallingTimes()
    {
        return rtaCallingTimes;
    }
    inline size_t ReadRTAControl()
    {
        return rtaControl;
    }

    /**
     * @brief this class modify everything in RTA_BASE to accomadate class DAG_Model
     *
     */
    class RTA_DAG_Model
    {
    private:
        DAG_Model &dagModel_;

    public:
        RTA_DAG_Model(DAG_Model &dagModel) : dagModel_(dagModel) {}

        double RTA_Common_Warm(double beginTime, int index)
        {
            CoutError("This function should not be used: RTA_Common_Warm, RTA_DAG_Model!");
            return -1;
        }

        VectorDynamic ResponseTimeOfTaskSet(const VectorDynamic &warmStart)
        {
            return ResponseTimeOfTaskSet();
        }
        VectorDynamic ResponseTimeOfTaskSet()
        {
            BeginTimer(__func__);
            IncrementCallingTimes();
            // prepare input
            std::stringstream tasksInput;
            tasksInput << ConvertTasksetToCsv();
            std::stringstream dagInput;
            dagInput << ConvertDAGsToCsv();
            std::stringstream abortsInput;
            tbb::task_scheduler_init init(tbb::task_scheduler_init::automatic);

            NP::Scheduling_problem<dtime_t> problem{
                NP::parse_file<dtime_t>(tasksInput),
                NP::parse_dag_file(dagInput),
                NP::parse_abort_file<dtime_t>(abortsInput),
                static_cast<unsigned int>(coreNumberAva)};

            // Set common analysis options
            NP::Analysis_options opts;
            opts.timeout = 0;
            opts.max_depth = 0;
            opts.early_exit = true;
            opts.num_buckets = problem.jobs.size();
            opts.be_naive = 0;

            // Actually call the analysis engine
            auto space = NP::Global::State_space<dtime_t>::explore(problem, opts);

            // Extract the analysis results
            TaskSetInfoDerived tasksInfo(dagModel_.tasks);
            VectorDynamic rta = GenerateVectorDynamic(tasksInfo.variableDimension);

            if (space.is_schedulable())
            {
                for (const auto &j : problem.jobs)
                {
                    NP::Interval<dtime_t> finish = space.get_finish_times(j);
                    LLint uniqueId = j.get_job_id();
                    rta(uniqueId) = (std::max<long long>(0, (finish.from() - j.earliest_arrival())));
                }
            }
            else
            {
                for (long int i = 0; i < rta.rows(); i++)
                {
                    rta(i) = INT32_MAX;
                }
            }
            EndTimer(__func__);
            return rta;
        }
        bool CheckSchedulability(VectorDynamic warmStart,
                                 bool whetherPrint = false, double tol = 0)
        {
            return CheckSchedulability();
        }
        bool CheckSchedulability(bool whetherPrint = false)
        {
            VectorDynamic rta = ResponseTimeOfTaskSet();
            return CheckSchedulabilityDirect(rta);
        }

        bool CheckSchedulabilityDirect(const VectorDynamic &rta)
        {
            TaskSetInfoDerived tasksInfo(dagModel_.tasks);
            return ExamDDL_Feasibility(dagModel_, tasksInfo, rta);
        }

        // used for schedulability check based on Nasri 19ECRTS
        std::string ConvertTasksetToCsv(bool saveOnDisk = false)
        {
            RegularTaskSystem::TaskSetInfoDerived tasksInfo(dagModel_.tasks);
            std::string taskSetStr = "Task ID,     Job ID,          Arrival min,          Arrival max,             Cost min,             Cost max,             Deadline,             Priority\n";
            for (int taskId = 0; taskId < dagModel_.tasks.size(); taskId++)
            {
                for (LLint jobId = 0; jobId < tasksInfo.sizeOfVariables[taskId]; jobId++)
                {
                    JobCEC jobCEC{taskId, jobId};
                    taskSetStr += std::to_string(taskId) + ", " + std::to_string(GetJobUniqueId(jobCEC, tasksInfo)) + ", " +
                                  std::to_string((LLint)GetActivationTime(jobCEC, tasksInfo)) + ", " + std::to_string((LLint)GetActivationTime(jobCEC, tasksInfo)) + ", " +
                                  std::to_string((LLint)GetExecutionTime(jobCEC, tasksInfo)) + ", " + std::to_string((LLint)GetExecutionTime(jobCEC, tasksInfo)) + ", " +
                                  std::to_string((LLint)GetDeadline(jobCEC, tasksInfo)) + ", " + std::to_string((LLint)GetDeadline(jobCEC, tasksInfo)) + "\n";
                }
            }
            if (saveOnDisk)
            {
                std::ofstream out("outputTask.csv");
                out << taskSetStr;
                out.close();
            }
            return taskSetStr;
        }
        // used for schedulability check based on Nasri 19ECRTS
        std::string ConvertDAGsToCsv(bool saveOnDisk = false)
        {
            TaskSetInfoDerived tasksInfo(dagModel_.tasks);
            std::string dependStr = "Predecessor TID,	Predecessor JID,	Successor TID, Successor JID\n";
            for (int taskId = 0; taskId < dagModel_.tasks.size(); taskId++)
            {
                for (LLint jobId = 1; jobId < tasksInfo.sizeOfVariables[taskId]; jobId++)
                {
                    JobCEC jobCEC{taskId, jobId - 1};
                    dependStr += std::to_string(taskId) + ", " + std::to_string(GetJobUniqueId(jobCEC, tasksInfo)) + ", ";
                    jobCEC.jobId = jobId;
                    dependStr += std::to_string(taskId) + ", " + std::to_string(GetJobUniqueId(jobCEC, tasksInfo)) + "\n";
                }
            }
            if (saveOnDisk)
            {
                std::ofstream out("outputDAG.csv");
                out << dependStr;
                out.close();
            }
            return dependStr;
        }
    };
}