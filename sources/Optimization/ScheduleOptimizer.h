#ifndef OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#define OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include "sources/Utils/BatchUtils.h"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

namespace DAG_SPACE
{
    class ScheduleOptimizer
    {
    public:
        ScheduleOptimizer()
        {
            reset();
        }
        void reset()
        {
            // env_.end();
            // model_.end();
            // cplex_solver_.end();
            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_solver_ = IloCplex(env_);
            num_variables_ = 0;
            num_hyper_periods_ = 0;
        }
        void Optimize(DAG_Model &dagTasks, ScheduleResult &result)
        {
            reset();
            setScheduleResult(result);
            TaskSetInfoDerived taskInfo(dagTasks.tasks);
            setTaskInfo(taskInfo);
            AddVariables();
            AddDBFConstraints();
            AddDDLConstraints();
            AddCauseEffectiveChainConstraints();
            AddObjectives();

            cplex_solver_.extract(model_);
            cplex_solver_.solve();

            auto status = cplex_solver_.getStatus();
            IloNumArray values_optimized(env_, 0);
            cplex_solver_.getValues(var_array_, values_optimized);
            GenerateOptimizedResult(values_optimized, dagTasks);
            std::cout << "Values are :" << values_optimized << "\n";
            std::cout << status << " solution found: " << cplex_solver_.getObjValue() << "\n";
        }

        void AddVariables()
        {
            num_variables_ = tasksInfo_.variableDimension;
            var_array_ = IloNumVarArray(env_, num_variables_, 0, tasksInfo_.hyperPeriod, IloNumVar::Float);
        }
        void AddDBFConstraints()
        {
            std::unordered_map<int, std::vector<JobCEC>> processor_job_map;
            int processor_id = 0;
            for (int i = 0; i < num_variables_; i++)
            {
                // TODO: need to support multiple processor id
                processor_id = 0;
                auto job = GetJobCECFromUniqueId(i, tasksInfo_);
                if (processor_job_map.count(processor_id) == 0)
                {
                    processor_job_map[processor_id] = std::vector<JobCEC>{};
                }
                processor_job_map[processor_id].push_back(job);
            }
            for (auto &pair : processor_job_map)
            {
                std::sort(pair.second.begin(), pair.second.end(),
                          [this](auto a, auto b) -> bool
                          { return GetStartTime(a, result_to_be_optimized_.startTimeVector_, tasksInfo_) <
                                   GetStartTime(b, result_to_be_optimized_.startTimeVector_, tasksInfo_); });
                if (pair.second.size() > 1)
                {
                    int pre_job_id = GetJobUniqueId(pair.second[0], tasksInfo_);
                    int cur_job_id;
                    for (auto j = 1u; j < pair.second.size(); j++)
                    {
                        cur_job_id = GetJobUniqueId(pair.second[j], tasksInfo_);
                        model_.add(var_array_[pre_job_id] + tasksInfo_.tasks[pair.second[j - 1].taskId].executionTime <= var_array_[cur_job_id]);
                        pre_job_id = cur_job_id;
                    }
                }
            }
        }
        void AddDDLConstraints()
        {
            for (int i = 0; i < num_variables_; i++)
            {
                model_.add(var_array_[i] >= GetActivationTime(GetJobCECFromUniqueId(i, tasksInfo_), tasksInfo_));
                model_.add(var_array_[i] + GetExecutionTime(i, tasksInfo_) <= GetDeadline(GetJobCECFromUniqueId(i, tasksInfo_), tasksInfo_));
            }
        }
        void AddCauseEffectiveChainConstraints()
        {
            num_hyper_periods_ = 1; // TODO: need to support multiple hyper periods
        }
        void AddObjectives()
        {
            IloExpr rtda_expression(env_);
            rtda_expression += var_array_[4] + tasksInfo_.tasks[2].executionTime - var_array_[0];
            rtda_expression += var_array_[4] + tasksInfo_.tasks[2].executionTime + tasksInfo_.hyperPeriod - var_array_[1];
            model_.add(IloMinimize(env_, rtda_expression));
            rtda_expression.end();
        }
        void GenerateOptimizedResult(IloNumArray &values_optimized, DAG_Model &dagTasks)
        {
            result_after_optimization_ = result_to_be_optimized_;
            VectorDynamic start_time(num_variables_);
            for (int i = 0; i < num_variables_; i++)
            {
                start_time(i) = values_optimized[i];
            }

            Values initialEstimateFG = GenerateInitialFG(start_time, tasksInfo_);
            std::vector<RTDA> rtda_vector;
            for (auto chain : dagTasks.chains_)
            {
                auto res = GetRTDAFromSingleJob(tasksInfo_, chain, initialEstimateFG);
                RTDA resM = GetMaxRTDA(res);
                if (resM.reactionTime == -1 || resM.dataAge == -1)
                {
                    return;
                }
                rtda_vector.push_back(resM);
            }
            RTDA rtda_optimized(0, 0);
            for (auto rtda : rtda_vector)
            {
                rtda_optimized.reactionTime += rtda.reactionTime;
                rtda_optimized.dataAge += rtda.dataAge;
            }
            if (ObjRTDA(rtda_optimized) < ObjRTDA(result_after_optimization_.rtda_))
            {
                result_after_optimization_.rtda_ = rtda_optimized;
                result_after_optimization_.obj_ = ObjRTDA(rtda_optimized);
            }
        }
        void setScheduleResult(ScheduleResult &res)
        {
            result_to_be_optimized_ = res;
        }
        ScheduleResult getOptimizedResult()
        {
            return result_after_optimization_;
        }
        void setTaskInfo(TaskSetInfoDerived &info)
        {
            tasksInfo_ = info;
        }
        void print()
        {
            std::cout << "ScheduleOptimizer print something!!!\n";
        }

    private:
        IloEnv env_;
        IloModel model_;
        IloCplex cplex_solver_;
        IloNumVarArray var_array_;
        ScheduleResult result_to_be_optimized_;
        ScheduleResult result_after_optimization_;
        TaskSetInfoDerived tasksInfo_;
        int num_variables_;
        int num_hyper_periods_;
    };
} // namespace DAG_SPACE
#endif // OPTIMIZATION_SCHEDULE_OPTIMIZER_H_