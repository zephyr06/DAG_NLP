#ifndef OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#define OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/OptimizeOrderUtils.h" // ScheduleResult
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

namespace OrderOptDAG_SPACE
{
    class ScheduleOptimizer
    {
    public:
        ScheduleOptimizer()
        {
            env_.end();
        }

        // function Optimize() will optimize RTDA
        void Optimize(DAG_Model &dagTasks, ScheduleResult &result)
        {
            // new environment, model, variables and solver
            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_solver_ = IloCplex(env_);
            p_dagTasks_ = nullptr;
            cplex_solver_.setOut(env_.getNullStream());

            setScheduleResult(result);
            setDagTasks(dagTasks);
            TaskSetInfoDerived tasksInfo(dagTasks.tasks);
            setTasksInfo(tasksInfo);
            AddVariables();
            AddDBFConstraints();
            AddDDLConstraints();
            if (considerSensorFusion)
            {
                AddSensorFusionConstraints();
            }
            AddObjectives(); // will also add cause effective chain constraints

            cplex_solver_.extract(model_);
            bool found_feasible_solution = cplex_solver_.solve();

            result_after_optimization_ = result_to_be_optimized_;
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
                double optimized_obj = cplex_solver_.getObjValue();
                if (optimized_obj < result_to_be_optimized_.obj_)
                {
                    result_after_optimization_.obj_ = optimized_obj;
                    UpdateOptimizedResult(values_optimized);
                }
            }

            // release memory
            cplex_solver_.end();
            var_array_.end();
            model_.end();
            env_.end();
        }

        // function OptimizeObjWeighted() will optimize weighted objectives
        // TODO(Dong): the primary job of ScheduleOptimizer is taking a job order and return a startTimeVector, constructing a ScheduleResult struct is confusing for other users because they don't know how to construct it properly
        void OptimizeObjWeighted(DAG_Model &dagTasks, ScheduleResult &result)
        {
            // new environment, model, variables and solver
            env_ = IloEnv();
            model_ = IloModel(env_);
            cplex_solver_ = IloCplex(env_);
            p_dagTasks_ = nullptr;
            cplex_solver_.setOut(env_.getNullStream());

            setScheduleResult(result);
            setDagTasks(dagTasks);
            TaskSetInfoDerived tasksInfo(dagTasks.tasks);
            setTasksInfo(tasksInfo);
            AddVariables();
            AddDBFConstraints();
            AddDDLConstraints();
            // cause effective chain constraints will be added together with objectives
            // Sensor Fusion is added as part of weighted objs
            AddWeightedObjectives();

            BeginTimer("LP_Solve");
            cplex_solver_.extract(model_);
            bool found_feasible_solution = cplex_solver_.solve();
            EndTimer("LP_Solve");

            result_after_optimization_ = result_to_be_optimized_;
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
                double optimized_obj_weighted = cplex_solver_.getObjValue();
                if (optimized_obj_weighted < result_to_be_optimized_.objWeighted_)
                {
                    result_after_optimization_.objWeighted_ = optimized_obj_weighted;
                    UpdateOptimizedResult(values_optimized);
                }
            }

            // release memory
            cplex_solver_.end();
            var_array_.end();
            model_.end();
            env_.end();
        }

        void print()
        {
            std::cout << "ScheduleOptimizer print something!!!\n";
        }

        // TODO(Dong): similar as above, it should not return ScheduleResult unless there's a good reason
        ScheduleResult getOptimizedResult()
        {
            return result_after_optimization_;
        }

    protected:
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
                AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);
            }
        }

        void AddCauseEffectiveChainConstraintsFromReactMap(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map)
        {
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

        void AddSensorFusionConstraints()
        {
            IloNumVar max_sensor_fusion_interval = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "MaxSensorFusionInterval");
            for (auto itr = p_dagTasks_->mapPrev.begin(); itr != p_dagTasks_->mapPrev.end(); itr++)
            {
                if (itr->second.size() < 2)
                {
                    continue;
                }
                std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map = GetSensorMapFromSingleJob(tasksInfo_, itr->first, itr->second, result_to_be_optimized_.startTimeVector_);

                for (auto pair : sensor_map)
                {
                    auto &precede_jobs = pair.second;
                    if (precede_jobs.size() < 2)
                    {
                        continue;
                    }
                    for (uint i = 0; i < precede_jobs.size(); i++)
                    {
                        for (uint j = i + 1; j < precede_jobs.size(); j++)
                        {
                            model_.add(max_sensor_fusion_interval >= (GetFinishTimeExpression(precede_jobs[i]) - GetFinishTimeExpression(precede_jobs[j])));
                            model_.add(max_sensor_fusion_interval >= (GetFinishTimeExpression(precede_jobs[j]) - GetFinishTimeExpression(precede_jobs[i])));
                        }
                    }
                    auto succeed_job = pair.first;
                    for (auto precede_job : precede_jobs)
                    {
                        model_.add(GetFinishTimeExpression(precede_job) <= GetStartTimeExpression(succeed_job));
                        precede_job.jobId++;
                        model_.add(GetFinishTimeExpression(precede_job) >= GetStartTimeExpression(succeed_job) + kCplexInequalityThreshold);
                    }
                }
            }
            model_.add(max_sensor_fusion_interval <= sensorFusionTolerance);
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

                // add cause effective chain constraints together with objectives to save time
                AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);

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

        void AddWeightedObjectives()
        {
            IloExpr obj_weighted_expr(env_);
            std::stringstream var_name;
            int chain_count = 0;
            LLint hyper_period = tasksInfo_.hyperPeriod;
            const TaskSet &tasks = tasksInfo_.tasks;
            IloExpr overall_rtda_expr(env_);
            IloExpr overall_sensor_fusion_expr(env_);

            // add obj and constraints related to RTDA
            for (auto chain : p_dagTasks_->chains_)
            {
                var_name << "Chain_" << chain_count << "_RT";
                auto theta_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                var_name.str("");
                var_name << "Chain_" << chain_count << "_DA";
                auto theta_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                var_name.str("");
                auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, result_to_be_optimized_.startTimeVector_);

                // add cause effective chain constraints together with objectives to save time
                AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);

                LLint total_start_jobs = hyper_period / tasks[chain[0]].period + 1;
                for (LLint start_instance_index = 0; start_instance_index <= total_start_jobs; start_instance_index++)
                {
                    JobCEC start_job = {chain[0], (start_instance_index)};
                    auto &react_chain = react_chain_map[start_job];
                    JobCEC first_react_job = react_chain.back();
                    auto reaction_time_cur_job_expr = GetFinishTimeExpression(first_react_job) - GetStartTimeExpression(start_job);
                    model_.add(theta_rt >= reaction_time_cur_job_expr);
                    overall_rtda_expr += reaction_time_cur_job_expr;

                    JobCEC last_start_job = {chain[0], (start_instance_index - 1)};
                    if (start_instance_index > 0 && react_chain_map[last_start_job].back() != first_react_job && first_react_job.jobId > 0)
                    {
                        JobCEC last_react_job = first_react_job;
                        last_react_job.jobId--;
                        auto data_age_last_job_expr = GetFinishTimeExpression(last_react_job) - GetStartTimeExpression(last_start_job);
                        model_.add(theta_da >= data_age_last_job_expr);
                        overall_rtda_expr += data_age_last_job_expr;
                    }
                }
                if (considerSensorFusion == 0)
                {
                    // Optmize RTDA: obj = max_RTs + max_DAs + overallRTDA * weightInMpRTDA
                    obj_weighted_expr += theta_rt;
                    obj_weighted_expr += theta_da;
                }
                else
                {
                    // only used in RTSS21IC experiment
                    // Optimize Sensor Fusion: obj = overallRTDA * someWeight + overallSensorFusion * someWeight +
                    // {Barrier(max(RT)) * w_punish + Barrier(max(DA)) * w_punish}**for_every_chain + Barrier(max(SensorFusion)) * w_punish
                    var_name << "Barrier_Chain_" << chain_count << "_RT";
                    auto barrier_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                    var_name.str("");
                    var_name << "Barrier_Chain_" << chain_count << "_DA";
                    auto barrier_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                    var_name.str("");
                    model_.add(barrier_rt >= 0);
                    model_.add(barrier_rt >= theta_rt - freshTol);
                    model_.add(barrier_da >= 0);
                    model_.add(barrier_da >= theta_da - freshTol);
                    obj_weighted_expr += barrier_rt * weightInMpRTDAPunish;
                    obj_weighted_expr += barrier_da * weightInMpRTDAPunish;
                }
            }
            obj_weighted_expr += overall_rtda_expr * weightInMpRTDA;

            // add obj and constraints related to sensor fusion
            if (considerSensorFusion)
            {
                IloNumVar max_sensor_fusion_interval = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "MaxSensorFusionInterval");
                LLint sensor_fusion_interval_count = 0;
                for (auto itr = p_dagTasks_->mapPrev.begin(); itr != p_dagTasks_->mapPrev.end(); itr++)
                {
                    if (itr->second.size() < 2)
                    {
                        continue;
                    }
                    std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map = GetSensorMapFromSingleJob(tasksInfo_, itr->first, itr->second, result_to_be_optimized_.startTimeVector_);

                    for (auto pair : sensor_map)
                    {
                        auto &precede_jobs = pair.second;
                        if (precede_jobs.size() < 2)
                        {
                            continue;
                        }
                        var_name << "Sensor_Fusion_Interval_" << sensor_fusion_interval_count++;
                        auto cur_sensor_fusion_interval = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
                        var_name.str("");
                        for (uint i = 0; i < precede_jobs.size(); i++)
                        {
                            for (uint j = i + 1; j < precede_jobs.size(); j++)
                            {
                                model_.add(cur_sensor_fusion_interval >= (GetFinishTimeExpression(precede_jobs[i]) - GetFinishTimeExpression(precede_jobs[j])));
                                model_.add(cur_sensor_fusion_interval >= (GetFinishTimeExpression(precede_jobs[j]) - GetFinishTimeExpression(precede_jobs[i])));
                            }
                        }
                        model_.add(max_sensor_fusion_interval >= cur_sensor_fusion_interval);
                        overall_sensor_fusion_expr += cur_sensor_fusion_interval;

                        // add sensor fusion constraints
                        auto succeed_job = pair.first;
                        for (auto precede_job : precede_jobs)
                        {
                            model_.add(GetFinishTimeExpression(precede_job) <= GetStartTimeExpression(succeed_job));
                            precede_job.jobId++;
                            model_.add(GetFinishTimeExpression(precede_job) >= GetStartTimeExpression(succeed_job) + kCplexInequalityThreshold);
                        }
                    }
                }

                auto barrier_sensor_fusion = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "Barrier_Sensor_Fusion");
                model_.add(barrier_sensor_fusion >= 0);
                model_.add(barrier_sensor_fusion >= max_sensor_fusion_interval - sensorFusionTolerance);
                obj_weighted_expr += barrier_sensor_fusion * weightInMpSfPunish;
                obj_weighted_expr += overall_sensor_fusion_expr * weightInMpSf;
            }

            model_.add(IloMinimize(env_, obj_weighted_expr));
            obj_weighted_expr.end();
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

        void UpdateOptimizedResult(IloNumArray &values_optimized)
        {
            VectorDynamic start_time(num_variables_);
            for (int i = 0; i < num_variables_; i++)
            {
                start_time(i) = values_optimized[i];
            }

            JobOrderMultiCore jobOrderRef(tasksInfo_, start_time);
            SFOrder sfOrder(tasksInfo_, start_time);
            result_after_optimization_.startTimeVector_ = start_time;
            result_after_optimization_.jobOrder_ = jobOrderRef;
            result_after_optimization_.sfOrder_ = sfOrder;
        }

        void setScheduleResult(ScheduleResult &res)
        {
            result_to_be_optimized_ = res;
        }

        inline void setDagTasks(DAG_Model &dagTasks)
        {
            p_dagTasks_ = &dagTasks;
        }

        void setTasksInfo(TaskSetInfoDerived &info)
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
    };
} // namespace OrderOptDAG_SPACE
#endif // OPTIMIZATION_SCHEDULE_OPTIMIZER_H_