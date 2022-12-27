#include "sources/Optimization/SFOrderLPOptimizer.h"

#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include "sources/Utils/Parameters.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"

namespace OrderOptDAG_SPACE
{
    void SFOrderLPOptimizer::Init()
    {
        if (hasBeenInitialized_)
        {
            this->ClearMemory();
        }
        TaskSetInfoDerived tasksInfo(dagTasks_.tasks);
        setTasksInfo(tasksInfo);
        env_ = IloEnv();
        model_ = IloModel(env_);
        cplexSolver_ = IloCplex(env_);
        cplexSolver_.setOut(env_.getNullStream());
        setOptimizedStartTimeVector();

        hasBeenInitialized_ = true;
    }

    void SFOrderLPOptimizer::ClearMemory()
    {
        // release memory
        if (hasBeenInitialized_)
        {
            cplexSolver_.end();
            model_.end();
            env_.end();
            hasBeenInitialized_ = false;
        }
    }

    inline void SFOrderLPOptimizer::setOptimizedStartTimeVector()
    {
        optimizedStartTimeVector_ = GenerateVectorDynamic(tasksInfo_.variableDimension);
    }

    // function Optimize() will optimize RTDA
    void SFOrderLPOptimizer::Optimize(const VectorDynamic &initialStartTimeVector, const std::vector<uint> &processorJobVec)
    {
        // new environment, model, variables and solver
        env_ = IloEnv();
        model_ = IloModel(env_);
        cplexSolver_ = IloCplex(env_);
        cplexSolver_.setOut(env_.getNullStream());

        setInitialStartTimeVector(initialStartTimeVector);
        setOptimizedStartTimeVector(initialStartTimeVector);
        setProcessorJobVec(processorJobVec);
        TaskSetInfoDerived tasksInfo(dagTasks_.tasks);
        setTasksInfo(tasksInfo);

        AddVariables();
        AddDBFConstraints();
        AddDDLConstraints();
        AddObjectives();

        cplexSolver_.extract(model_);
        bool found_feasible_solution = cplexSolver_.solve();

        IloNumArray values_optimized(env_, numVariables_);
        if (found_feasible_solution)
        {
            auto status = cplexSolver_.getStatus();
            cplexSolver_.getValues(varArray_, values_optimized);
            if (GlobalVariablesDAGOpt::debugMode)
            {
                std::cout << "Values are :" << values_optimized << "\n";
                std::cout << status << " solution found: " << cplexSolver_.getObjValue() << "\n";
            }

            // TODO: need to find a way to pass the initial obj (the obj before optimization)
            // right now will update the optimized start time vec no matter whether find a better obj or not

            // double optimized_obj = cplexSolver_.getObjValue();
            // if (optimized_obj < result_to_be_optimized_.obj_)
            // {
            //     result_after_optimization_.obj_ = optimized_obj;
            //     UpdateOptimizedStartTimeVector(values_optimized);
            // }

            UpdateOptimizedStartTimeVector(values_optimized);
        }

        // release memory
        cplexSolver_.end();
        varArray_.end();
        model_.end();
        env_.end();
    }

    void SFOrderLPOptimizer::OptimizeWithJobOrder(const VectorDynamic &initialStartTimeVector, const std::vector<uint> &processorJobVec, const SFOrder &jobOrder)
    {
        // new environment, model, variables and solver
        env_ = IloEnv();
        model_ = IloModel(env_);
        cplexSolver_ = IloCplex(env_);
        cplexSolver_.setOut(env_.getNullStream());

        setInitialStartTimeVector(initialStartTimeVector);
        setOptimizedStartTimeVector(initialStartTimeVector);
        setProcessorJobVec(processorJobVec);
        TaskSetInfoDerived tasksInfo(dagTasks_.tasks);
        setTasksInfo(tasksInfo);
        AddVariables();
        AddDBFConstraints();
        AddDDLConstraints();
        AddJobOrderConstraints(jobOrder);
        AddObjectives();

        cplexSolver_.extract(model_);
        bool found_feasible_solution = cplexSolver_.solve();

        IloNumArray values_optimized(env_, numVariables_);
        if (found_feasible_solution)
        {
            auto status = cplexSolver_.getStatus();
            cplexSolver_.getValues(varArray_, values_optimized);
            if (GlobalVariablesDAGOpt::debugMode)
            {
                std::cout << "Values are :" << values_optimized << "\n";
                std::cout << status << " solution found: " << cplexSolver_.getObjValue() << "\n";
            }
            UpdateOptimizedStartTimeVector(values_optimized);
        }

        // release memory
        cplexSolver_.end();
        varArray_.end();
        model_.end();
        env_.end();
    }

    VectorDynamic SFOrderLPOptimizer::getOptimizedStartTimeVector()
    {
        return optimizedStartTimeVector_;
    }

    void SFOrderLPOptimizer::AddVariables()
    {
        numVariables_ = tasksInfo_.variableDimension;
        varArray_ = IloNumVarArray(env_, numVariables_, 0, tasksInfo_.hyperPeriod, IloNumVar::Float);
    }

    void SFOrderLPOptimizer::AddDBFConstraints()
    {
        std::unordered_map<int, std::vector<JobCEC>> processor_job_map;
        int processor_id = 0;
        for (int i = 0; i < numVariables_; i++)
        {
            if (!processorJobVec_.empty())
            {
                processor_id = processorJobVec_[i];
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
                      { return GetStartTime(a, initialStartTimeVector_, tasksInfo_) <
                               GetStartTime(b, initialStartTimeVector_, tasksInfo_); });
            if (pair.second.size() > 1)
            {
                int pre_job_id = GetJobUniqueId(pair.second[0], tasksInfo_);
                int cur_job_id;
                for (auto j = 1u; j < pair.second.size(); j++)
                {
                    cur_job_id = GetJobUniqueId(pair.second[j], tasksInfo_);
                    model_.add(varArray_[pre_job_id] + GetExecutionTime(pre_job_id, tasksInfo_) <= varArray_[cur_job_id]);
                    pre_job_id = cur_job_id;
                }
            }
        }
    }

    void SFOrderLPOptimizer::AddDDLConstraints()
    {
        for (int i = 0; i < numVariables_; i++)
        {
            model_.add(varArray_[i] >= GetActivationTime(GetJobCECFromUniqueId(i, tasksInfo_), tasksInfo_));
            model_.add(varArray_[i] + GetExecutionTime(i, tasksInfo_) <= GetDeadline(GetJobCECFromUniqueId(i, tasksInfo_), tasksInfo_));
        }
    }

    void SFOrderLPOptimizer::AddJobOrderConstraints(const SFOrder &jobOrder)
    {
        const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
        for (uint i = 1; i < instanceOrder.size(); i++)
        {
            auto instCurr = instanceOrder[i];
            auto instPrev = instanceOrder[i - 1];
            int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo_);
            int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo_);
            if (instPrev.type == 's')
            {
                if (instCurr.type == 's')
                {
                    model_.add(varArray_[globalIdCurr] >= varArray_[globalIdPrev]);
                }
                else // type == 'f'
                {
                    model_.add(varArray_[globalIdCurr] + GetExecutionTime(instCurr.job, tasksInfo_) >= varArray_[globalIdPrev]);
                }
            }
            else // type == 'f'
            {
                if (instCurr.type == 's')
                {
                    model_.add(varArray_[globalIdCurr] >= varArray_[globalIdPrev] + GetExecutionTime(instPrev.job, tasksInfo_));
                }
                else // type == 'f'
                {
                    model_.add(varArray_[globalIdCurr] + GetExecutionTime(instCurr.job, tasksInfo_) >= varArray_[globalIdPrev] + GetExecutionTime(instPrev.job, tasksInfo_));
                }
            }
        }
    }

    void SFOrderLPOptimizer::AddCauseEffectiveChainConstraints()
    {
        for (auto chain : dagTasks_.chains_)
        {
            auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, initialStartTimeVector_);
            AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);
        }
    }

    void SFOrderLPOptimizer::AddCauseEffectiveChainConstraintsFromReactMap(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map)
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
                        model_.add(GetStartTimeExpression(pre_job_of_same_task) <= GetFinishTimeExpression(pre_job_in_chain) - GlobalVariablesDAGOpt::kCplexInequalityThreshold);
                    }
                    pre_job_in_chain = cur_job;
                }
            }
        }
    }

    void SFOrderLPOptimizer::AddSensorFusionConstraints()
    {
        IloNumVar max_sensor_fusion_interval = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "MaxSensorFusionInterval");
        for (auto itr = dagTasks_.mapPrev.begin(); itr != dagTasks_.mapPrev.end(); itr++)
        {
            if (itr->second.size() < 2)
            {
                continue;
            }
            std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map = GetSensorMapFromSingleJob(tasksInfo_, itr->first, itr->second, initialStartTimeVector_);

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
                    model_.add(GetFinishTimeExpression(precede_job) >= GetStartTimeExpression(succeed_job) + GlobalVariablesDAGOpt::kCplexInequalityThreshold);
                }
            }
        }
        model_.add(max_sensor_fusion_interval <= GlobalVariablesDAGOpt::sensorFusionTolerance);
    }

    void SFOrderLPOptimizer::AddObjectives()
    {
        if (!useWeightedObj_)
        {
            if (GlobalVariablesDAGOpt::considerSensorFusion)
            {
                AddSensorFusionConstraints();
            }
            AddNormalObjectives(); // will also add cause effective chain constraints
        }
        else
        {
            // cause effective chain constraints will be added together with objectives
            // Sensor Fusion is added as part of weighted objs
            AddWeightedObjectives();
        }
    }
    void SFOrderLPOptimizer::AddNormalObjectives()
    {
        IloExpr rtda_expression(env_);
        std::stringstream var_name;
        int chain_count = 0;
        LLint hyper_period = tasksInfo_.hyperPeriod;
        const TaskSet &tasks = tasksInfo_.tasks;

        for (auto chain : dagTasks_.chains_)
        {
            var_name << "Chain_" << chain_count << "_RT";
            auto theta_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
            var_name.str("");
            var_name << "Chain_" << chain_count << "_DA";
            auto theta_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
            var_name.str("");
            auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, initialStartTimeVector_);

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

    void SFOrderLPOptimizer::AddWeightedObjectives()
    {
        IloExpr obj_weighted_expr(env_);
        std::stringstream var_name;
        int chain_count = 0;
        LLint hyper_period = tasksInfo_.hyperPeriod;
        const TaskSet &tasks = tasksInfo_.tasks;
        IloExpr overall_rtda_expr(env_);
        IloExpr overall_sensor_fusion_expr(env_);

        // add obj and constraints related to RTDA
        for (auto chain : dagTasks_.chains_)
        {
            var_name << "Chain_" << chain_count << "_RT";
            auto theta_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
            var_name.str("");
            var_name << "Chain_" << chain_count << "_DA";
            auto theta_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.str().c_str());
            var_name.str("");
            auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_, chain, initialStartTimeVector_);

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
            if (GlobalVariablesDAGOpt::considerSensorFusion == 0)
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
                model_.add(barrier_rt >= theta_rt - GlobalVariablesDAGOpt::freshTol);
                model_.add(barrier_da >= 0);
                model_.add(barrier_da >= theta_da - GlobalVariablesDAGOpt::freshTol);
                obj_weighted_expr += barrier_rt * GlobalVariablesDAGOpt::weightInMpRTDAPunish;
                obj_weighted_expr += barrier_da * GlobalVariablesDAGOpt::weightInMpRTDAPunish;
            }
        }
        obj_weighted_expr += overall_rtda_expr * GlobalVariablesDAGOpt::weightInMpRTDA;

        // add obj and constraints related to sensor fusion
        if (GlobalVariablesDAGOpt::considerSensorFusion)
        {
            IloNumVar max_sensor_fusion_interval = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "MaxSensorFusionInterval");
            LLint sensor_fusion_interval_count = 0;
            for (auto itr = dagTasks_.mapPrev.begin(); itr != dagTasks_.mapPrev.end(); itr++)
            {
                if (itr->second.size() < 2)
                {
                    continue;
                }
                std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map = GetSensorMapFromSingleJob(tasksInfo_, itr->first, itr->second, initialStartTimeVector_);

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
                        model_.add(GetFinishTimeExpression(precede_job) >= GetStartTimeExpression(succeed_job) + GlobalVariablesDAGOpt::kCplexInequalityThreshold);
                    }
                }
            }

            auto barrier_sensor_fusion = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, "Barrier_Sensor_Fusion");
            model_.add(barrier_sensor_fusion >= 0);
            model_.add(barrier_sensor_fusion >= max_sensor_fusion_interval - GlobalVariablesDAGOpt::sensorFusionTolerance);
            obj_weighted_expr += barrier_sensor_fusion * GlobalVariablesDAGOpt::weightInMpSfPunish;
            obj_weighted_expr += overall_sensor_fusion_expr * GlobalVariablesDAGOpt::weightInMpSf;
        }

        model_.add(IloMinimize(env_, obj_weighted_expr));
        obj_weighted_expr.end();
    }

    IloExpr SFOrderLPOptimizer::GetStartTimeExpression(JobCEC &jobCEC)
    {
        IloExpr exp(env_);
        if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo_.N)
        {
            CoutError("GetStartTime receives invalid jobCEC!");
        }
        int jobNumInHyperPeriod = tasksInfo_.hyperPeriod / tasksInfo_.tasks[jobCEC.taskId].period;
        exp += varArray_[GetJobUniqueId(jobCEC, tasksInfo_)];
        exp += jobCEC.jobId / jobNumInHyperPeriod * tasksInfo_.hyperPeriod;
        return exp;
    }

    IloExpr SFOrderLPOptimizer::GetFinishTimeExpression(JobCEC &jobCEC)
    {
        return GetStartTimeExpression(jobCEC) + GetExecutionTime(jobCEC, tasksInfo_);
    }

    void SFOrderLPOptimizer::UpdateOptimizedStartTimeVector(IloNumArray &values_optimized)
    {
        for (int i = 0; i < numVariables_; i++)
        {
            optimizedStartTimeVector_(i) = values_optimized[i];
        }
    }

    inline void SFOrderLPOptimizer::setInitialStartTimeVector(const VectorDynamic &initialStartTimeVector)
    {
        initialStartTimeVector_ = initialStartTimeVector;
    }

    inline void SFOrderLPOptimizer::setOptimizedStartTimeVector(const VectorDynamic &optimizedStartTimeVector)
    {
        optimizedStartTimeVector_ = optimizedStartTimeVector;
    }

    inline void SFOrderLPOptimizer::setProcessorJobVec(const std::vector<uint> &processorJobVec)
    {
        processorJobVec_ = processorJobVec;
    }

    inline void SFOrderLPOptimizer::setTasksInfo(const TaskSetInfoDerived &info)
    {
        tasksInfo_ = info;
    }

} // namespace OrderOptDAG_SPACE