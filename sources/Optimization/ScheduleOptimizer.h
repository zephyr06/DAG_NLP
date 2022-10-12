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

        void Optimize(DAG_Model &dagTasks, ScheduleResult &result)
        {
            reset();
            setScheduleResult(result);
            setDagTasks(dagTasks);
            TaskSetInfoDerived taskInfo(dagTasks.tasks);
            setTaskInfo(taskInfo);
            AddVariables();
            AddDBFConstraints();
            AddDDLConstraints();
            AddCauseEffectiveChainConstraints();
            AddObjectives();

            cplex_solver_.extract(model_);
            bool found_feasible_solution = cplex_solver_.solve();

            IloNumArray values_optimized(env_, num_variables_);
            if (found_feasible_solution)
            {
                auto status = cplex_solver_.getStatus();
                cplex_solver_.getValues(var_array_, values_optimized);
                if (debugMode)
                {
                    std::cout << "Values are :" << values_optimized << "\n";
                    std::cout << status << " solution found: " << cplex_solver_.getObjValue() << "\n";
                }
            }
            GenerateOptimizedResult(values_optimized);
        }

        void print()
        {
            std::cout << "ScheduleOptimizer print something!!!\n";
        }

        ScheduleResult getOptimizedResult()
        {
            return result_after_optimization_;
        }

    protected:
        void reset()
        {
            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_solver_ = IloCplex(env_);
            num_variables_ = 0;
            num_hyper_periods_ = 0;
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
                if (!result_to_be_optimized_.processorJobVec_.empty())
                {
                    processor_id = result_to_be_optimized_.processorJobVec_[i];
                }
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
                        model_.add(var_array_[pre_job_id] + GetExecutionTime(pre_job_id, tasksInfo_) <= var_array_[cur_job_id]);
                        pre_job_id = cur_job_id;
                    }
                }
            }
            // TODO: need to support job order constraint ?
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
            for (auto chain : p_dagTasks_->chains_)
            {
                auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, result_to_be_optimized_.startTimeVector_);
                for (auto pair : react_chain_map)
                {
                    auto &react_chain = pair.second;
                    if (react_chain.size() > 1)
                    {
                        JobCEC pre_job_in_chain = react_chain[0];
                        for (auto i = 1u; i < react_chain.size(); i++)
                        {
                            JobCEC cur_job = react_chain[i];
                            JobCEC pre_job_of_same_task = cur_job;
                            pre_job_of_same_task.jobId--;
                            model_.add(GetFinishTimeExpression(pre_job_in_chain) <= GetStartTimeExpression(cur_job));
                            if (pre_job_of_same_task.jobId >= 0)
                            {
                                // Cplex only support weak inequality, a threshold is added to enforce strict inequality
                                model_.add(GetStartTimeExpression(pre_job_of_same_task) <= GetFinishTimeExpression(pre_job_in_chain) - kCplexInequalityThreshold);
                            }
                            pre_job_in_chain = cur_job;
                        }
                    }
                }
            }
        }

        void AddObjectives()
        {
            IloExpr rtda_expression(env_);
            std::stringstream var_name;
            int chain_count = 0;
            LLint hyper_period = tasksInfo_.hyperPeriod;
            const TaskSet &tasks = tasksInfo_.tasks;

            for (auto chain : p_dagTasks_->chains_)
            {
                var_name << "Chain_" << chain_count << "_RT";
                auto theta_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                var_name.str("");
                var_name << "Chain_" << chain_count << "_DA";
                auto theta_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                var_name.str("");
                auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, result_to_be_optimized_.startTimeVector_);

                LLint total_start_jobs = hyper_period / tasks[chain[0]].period + 1;
                for (LLint start_instance_index = 0; start_instance_index <= total_start_jobs; start_instance_index++)
                {
                    JobCEC start_job = {chain[0], (start_instance_index)};
                    auto &react_chain = react_chain_map[start_job];
                    JobCEC first_react_job = react_chain.back();
                    model_.add(theta_rt >= (GetFinishTimeExpression(first_react_job) - GetStartTimeExpression(start_job)));

                    JobCEC last_start_job = {chain[0], (start_instance_index - 1)};
                    if (start_instance_index > 0 && react_chain_map[last_start_job].back() != first_react_job && first_react_job.jobId > 0)
                    {
                        JobCEC last_react_job = first_react_job;
                        last_react_job.jobId--;
                        model_.add(theta_da >= (GetFinishTimeExpression(last_react_job) - GetStartTimeExpression(last_start_job)));
                    }
                }
                rtda_expression += theta_rt;
                rtda_expression += theta_da;
            }
            model_.add(IloMinimize(env_, rtda_expression));
            rtda_expression.end();
        }

        IloExpr GetStartTimeExpression(JobCEC &jobCEC)
        {
            IloExpr exp(env_);
            if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo_.N)
            {
                CoutError("GetStartTime receives invalid jobCEC!");
            }
            int jobNumInHyperPeriod = tasksInfo_.hyperPeriod / tasksInfo_.tasks[jobCEC.taskId].period;
            exp += var_array_[GetJobUniqueId(jobCEC, tasksInfo_)];
            exp += jobCEC.jobId / jobNumInHyperPeriod * tasksInfo_.hyperPeriod;
            return exp;
        }

        IloExpr GetFinishTimeExpression(JobCEC &jobCEC)
        {
            return GetStartTimeExpression(jobCEC) + GetExecutionTime(jobCEC, tasksInfo_);
        }

        void GenerateOptimizedResult(IloNumArray &values_optimized)
        {
            result_after_optimization_ = result_to_be_optimized_;
            VectorDynamic start_time(num_variables_);
            for (int i = 0; i < num_variables_; i++)
            {
                start_time(i) = values_optimized[i];
            }

            Values initialEstimateFG = GenerateInitialFG(start_time, tasksInfo_);
            std::vector<RTDA> rtda_vector;
            for (auto chain : p_dagTasks_->chains_)
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
                result_after_optimization_.startTimeVector_ = start_time;
            }
        }

        void setScheduleResult(ScheduleResult &res)
        {
            result_to_be_optimized_ = res;
        }

        inline void setDagTasks(DAG_Model &dagTasks)
        {
            p_dagTasks_ = &dagTasks;
        }

        void setTaskInfo(TaskSetInfoDerived &info)
        {
            tasksInfo_ = info;
        }

    private:
        IloEnv env_;
        IloModel model_;
        IloCplex cplex_solver_;
        IloNumVarArray var_array_;
        ScheduleResult result_to_be_optimized_;
        ScheduleResult result_after_optimization_;
        TaskSetInfoDerived tasksInfo_;
        DAG_Model *p_dagTasks_;
        int num_variables_;
        int num_hyper_periods_;
    };
} // namespace DAG_SPACE
#endif // OPTIMIZATION_SCHEDULE_OPTIMIZER_H_