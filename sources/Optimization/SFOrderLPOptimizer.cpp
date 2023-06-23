#include "sources/Optimization/SFOrderLPOptimizer.h"

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE {
void SFOrderLPOptimizer::Init() {
    if (hasBeenInitialized_) {
        this->ClearCplexMemory();
    }
    TaskSetInfoDerived tasksInfo(dagTasks_.tasks);
    setTasksInfo(tasksInfo);
    env_ = IloEnv();
    model_ = IloModel(env_);
    cplexSolver_ = IloCplex(env_);
    cplexSolver_.setOut(env_.getNullStream());
    setOptimizedStartTimeVector();
    processorJobVec_.clear();

    hasBeenInitialized_ = true;
}

void SFOrderLPOptimizer::ClearCplexMemory() {
    // release memory
    if (hasBeenInitialized_) {
        cplexSolver_.end();
        model_.end();
        env_.end();
        hasBeenInitialized_ = false;
    }
}

// function Optimize() will optimize RTDA
void SFOrderLPOptimizer::Optimize(const std::vector<uint> &processorJobVec) {
    // BeginTimer("Build_LP_Model");
    this->Init();
    setProcessorJobVec(processorJobVec);

    AddVariables();
    AddDBFConstraints();
    AddDDLConstraints();
    if (GlobalVariablesDAGOpt::EnableHardJobORder)
        AddJobOrderConstraints(sfOrder_);
    AddObjectives();
    cplexSolver_.extract(model_);
    // EndTimer("Build_LP_Model");

    // BeginTimer("Solve_LP");
    bool found_feasible_solution = cplexSolver_.solve();
    // EndTimer("Solve_LP");
    IloNumArray values_optimized(env_, numVariables_);
    if (found_feasible_solution) {
        auto status = cplexSolver_.getStatus();
        cplexSolver_.getValues(varArray_, values_optimized);
        if (GlobalVariablesDAGOpt::debugMode) {
            std::cout << "Values are :" << values_optimized << "\n";
            std::cout << status
                      << " solution found: " << cplexSolver_.getObjValue()
                      << "\n";
        }

        // TODO(Dong): need to find a way to pass the initial obj (the obj
        // before optimization) right now will update the optimized start time
        // vec no matter whether find a better obj or not

        // double optimized_obj = cplexSolver_.getObjValue();
        // if (optimized_obj < result_to_be_optimized_.obj_)
        // {
        //     result_after_optimization_.obj_ = optimized_obj;
        //     UpdateOptimizedStartTimeVector(values_optimized);
        // }

        UpdateOptimizedStartTimeVector(values_optimized);
    }
    if (GlobalVariablesDAGOpt::debugMode)
        WriteModelToFile();
    this->ClearCplexMemory();
}

VectorDynamic SFOrderLPOptimizer::getOptimizedStartTimeVector() {
    return optimizedStartTimeVector_;
}

void SFOrderLPOptimizer::AddVariables() {
    numVariables_ = tasksInfo_.variableDimension;
    varArray_ = IloNumVarArray(env_, numVariables_, 0, tasksInfo_.hyperPeriod,
                               IloNumVar::Float);
}

void SFOrderLPOptimizer::AddDBFConstraints() {
    // BeginTimer("LPAddDBFConstraints");
    std::vector<TimeInstance> prevInstancesEachProcessor;
    prevInstancesEachProcessor.reserve(processorNum_);
    for (int i = 0; i < processorNum_; i++) {
        prevInstancesEachProcessor.push_back(TimeInstance('n', JobCEC(0, 0)));
    }
    for (uint i = 0; i < sfOrder_.instanceOrder_.size(); i++) {
        JobCEC jobCurr = sfOrder_.instanceOrder_[i].job;
        int cur_job_id = GetJobUniqueId(jobCurr, tasksInfo_);
        int processor_id = processorJobVec_[cur_job_id];
        if (prevInstancesEachProcessor[processor_id].type != 'n') {
            JobCEC jobPrev = prevInstancesEachProcessor[processor_id].job;
            int pre_job_id = GetJobUniqueId(jobPrev, tasksInfo_);
            // FOR DEBUG ONLY
            // std::cout << "DBF: (" << jobPrev.ToString() << ", " <<
            // jobCurr.ToString() << "\n";
            if (jobPrev != jobCurr)
                model_.add(varArray_[pre_job_id] +
                               GetExecutionTime(pre_job_id, tasksInfo_) <=
                           varArray_[cur_job_id]);
        }
        prevInstancesEachProcessor[processor_id] = sfOrder_.instanceOrder_[i];
    }

    // EndTimer("LPAddDBFConstraints");
}

void SFOrderLPOptimizer::AddDDLConstraints() {
    for (int i = 0; i < numVariables_; i++) {
        model_.add(varArray_[i] >=
                   GetActivationTime(GetJobCECFromUniqueId(i, tasksInfo_),
                                     tasksInfo_));
        model_.add(
            varArray_[i] + GetExecutionTime(i, tasksInfo_) <=
            GetDeadline(GetJobCECFromUniqueId(i, tasksInfo_), tasksInfo_));
    }
}

void SFOrderLPOptimizer::AddJobOrderConstraints(const SFOrder &jobOrder) {
    double tolerance = GlobalVariablesDAGOpt::LPTolerance;
    const std::vector<TimeInstance> &instanceOrder = jobOrder.instanceOrder_;
    for (uint i = 1; i < instanceOrder.size(); i++) {
        auto instCurr = instanceOrder[i];
        auto instPrev = instanceOrder[i - 1];
        int globalIdCurr = GetJobUniqueId(instCurr.job, tasksInfo_);
        int globalIdPrev = GetJobUniqueId(instPrev.job, tasksInfo_);
        if (globalIdCurr == globalIdPrev)
            continue;
        if (instPrev.type == 's') {
            if (instCurr.type == 's') {
                if (instPrev.job.taskId <= instCurr.job.taskId)
                    model_.add(varArray_[globalIdCurr] >=
                               varArray_[globalIdPrev]);
                else
                    model_.add(varArray_[globalIdCurr] >=
                               varArray_[globalIdPrev] + tolerance);
            } else  // instCurr.type == 'f'
            {
                model_.add(varArray_[globalIdCurr] +
                               GetExecutionTime(instCurr.job, tasksInfo_) >=
                           varArray_[globalIdPrev] + tolerance);
            }
        } else  // instPrev.type == 'f'
        {
            if (instCurr.type == 's') {
                model_.add(varArray_[globalIdCurr] >=
                           varArray_[globalIdPrev] +
                               GetExecutionTime(instPrev.job, tasksInfo_));
            } else  // instCurr.type == 'f'
            {
                if (instPrev.job.taskId <= instCurr.job.taskId)
                    model_.add(varArray_[globalIdCurr] +
                                   GetExecutionTime(instCurr.job, tasksInfo_) >=
                               varArray_[globalIdPrev] +
                                   GetExecutionTime(instPrev.job, tasksInfo_));
                else
                    model_.add(varArray_[globalIdCurr] +
                                   GetExecutionTime(instCurr.job, tasksInfo_) >=
                               varArray_[globalIdPrev] +
                                   GetExecutionTime(instPrev.job, tasksInfo_) +
                                   tolerance);
            }
        }
    }
    if (GlobalVariablesDAGOpt::debugMode)
        WriteModelToFile("JobOrderConst.lp");
}

void SFOrderLPOptimizer::AddCauseEffectiveChainConstraintsFromReactMap(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map) {
    for (auto pair : react_chain_map) {
        auto &react_chain = pair.second;
        if (react_chain.size() > 1) {
            JobCEC pre_job_in_chain = react_chain[0];
            for (auto i = 1u; i < react_chain.size(); i++) {
                JobCEC cur_job = react_chain[i];
                JobCEC pre_job_of_same_task = cur_job;
                pre_job_of_same_task.jobId--;
                model_.add(GetFinishTimeExpression(pre_job_in_chain) <=
                           GetStartTimeExpression(cur_job));
                if (pre_job_of_same_task.jobId >= 0) {
                    // Cplex only support weak inequality, a threshold is added
                    // to enforce strict inequality
                    model_.add(
                        GetStartTimeExpression(pre_job_of_same_task) <=
                        GetFinishTimeExpression(pre_job_in_chain) -
                            GlobalVariablesDAGOpt::kCplexInequalityThreshold);
                }
                pre_job_in_chain = cur_job;
            }
        }
    }
}

void SFOrderLPOptimizer::AddCauseEffectiveChainConstraintsFromDaMap(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &da_chain_map) {
    for (auto pair : da_chain_map) {
        auto &job_chain = pair.second;
        if (job_chain.size() > 1) {
            JobCEC sink_job = pair.first;
            for (auto i = int(job_chain.size()) - 2; i >= 0; i--) {
                JobCEC curr_job = job_chain[i];
                JobCEC curr_job_next(curr_job.taskId, curr_job.jobId + 1);
                model_.add(GetFinishTimeExpression(curr_job) <=
                           GetStartTimeExpression(sink_job));

                // Cplex only support weak inequality, a threshold is added to
                // enforce strict inequality
                model_.add(
                    GetStartTimeExpression(sink_job) +
                        GlobalVariablesDAGOpt::kCplexInequalityThreshold <=
                    GetFinishTimeExpression(curr_job_next));

                sink_job = curr_job;
            }
        }
    }
    if (GlobalVariablesDAGOpt::debugMode)
        WriteModelToFile("CauseEffectChainConst.lp");
}

void SFOrderLPOptimizer::AddSensorFusionConstraints() {
    IloNumVar max_sensor_fusion_interval = IloNumVar(
        env_, 0, IloInfinity, IloNumVar::Float, "MaxSensorFusionInterval");
    for (auto itr = dagTasks_.mapPrev.begin(); itr != dagTasks_.mapPrev.end();
         itr++) {
        if (itr->second.size() < 2) {
            continue;
        }
        std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map =
            GetSensorMapFromSingleJob(tasksInfo_, itr->first, itr->second,
                                      initialStartTimeVector_);

        for (auto pair : sensor_map) {
            auto &precede_jobs = pair.second;
            if (precede_jobs.size() < 2) {
                continue;
            }
            for (uint i = 0; i < precede_jobs.size(); i++) {
                for (uint j = i + 1; j < precede_jobs.size(); j++) {
                    model_.add(max_sensor_fusion_interval >=
                               (GetFinishTimeExpression(precede_jobs[i]) -
                                GetFinishTimeExpression(precede_jobs[j])));
                    model_.add(max_sensor_fusion_interval >=
                               (GetFinishTimeExpression(precede_jobs[j]) -
                                GetFinishTimeExpression(precede_jobs[i])));
                }
            }
            auto succeed_job = pair.first;
            for (auto precede_job : precede_jobs) {
                model_.add(GetFinishTimeExpression(precede_job) <=
                           GetStartTimeExpression(succeed_job));
                precede_job.jobId++;
                model_.add(
                    GetFinishTimeExpression(precede_job) >=
                    GetStartTimeExpression(succeed_job) +
                        GlobalVariablesDAGOpt::kCplexInequalityThreshold);
            }
        }
    }
    model_.add(max_sensor_fusion_interval <=
               GlobalVariablesDAGOpt::sensorFusionTolerance);
}

void SFOrderLPOptimizer::AddObjectives() {
    if (obj_type_trait_ == "ReactionTimeObj")
        AddReactionTimeObj();
    else if (obj_type_trait_ == "DataAgeObj")
        AddDataAgeObj();
    else if (obj_type_trait_ == "SensorFusionObj")
        AddSensorFusionObj();
    else
        CoutError("Unrecognized type_trait in AddObjectives()!");
}

void SFOrderLPOptimizer::AddNormalObjectives() {
    IloExpr rtda_expression(env_);
    std::stringstream var_name;
    int chain_count = 0;
    LLint hyper_period = tasksInfo_.hyperPeriod;
    const TaskSet &tasks = tasksInfo_.tasks;

    for (auto chain : dagTasks_.chains_) {
        var_name << "Chain_" << chain_count << "_RT";
        auto theta_rt = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float,
                                  var_name.str().c_str());
        var_name.str("");
        var_name << "Chain_" << chain_count << "_DA";
        auto theta_da = IloNumVar(env_, 0, IloInfinity, IloNumVar::Float,
                                  var_name.str().c_str());
        var_name.str("");
        // auto react_chain_map = GetRTDAReactChainsFromSingleJob(tasksInfo_,
        // chain, initialStartTimeVector_);
        auto react_chain_map =
            GetReactionChainMap(dagTasks_, tasksInfo_, sfOrder_, 1, chain, 1);

        // add cause effective chain constraints together with objectives to
        // save time
        AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);

        LLint total_start_jobs = hyper_period / tasks[chain[0]].period + 1;
        for (LLint start_instance_index = 0;
             start_instance_index <= total_start_jobs; start_instance_index++) {
            JobCEC start_job = {chain[0], (start_instance_index)};
            auto &react_chain = react_chain_map[start_job];
            JobCEC first_react_job = react_chain.back();
            model_.add(theta_rt >= (GetFinishTimeExpression(first_react_job) -
                                    GetStartTimeExpression(start_job)));

            JobCEC last_start_job = {chain[0], (start_instance_index - 1)};
            if (start_instance_index > 0 &&
                react_chain_map[last_start_job].back() != first_react_job &&
                first_react_job.jobId > 0) {
                JobCEC last_react_job = first_react_job;
                last_react_job.jobId--;
                model_.add(theta_da >=
                           (GetFinishTimeExpression(last_react_job) -
                            GetStartTimeExpression(last_start_job)));
            }
        }
        // Normal obj to optmize RTDA: obj = max_RTs + max_DAs
        rtda_expression += theta_rt;
        rtda_expression += theta_da;
    }
    model_.add(IloMinimize(env_, rtda_expression));
    rtda_expression.end();
}

void SFOrderLPOptimizer::AddReactionTimeObj() {
    IloExpr rtda_expression(env_);
    int chain_count = 0;
    for (auto chain : dagTasks_.chains_) {
        std::string var_name = "Chain_" + std::to_string(chain_count++) + "_RT";
        auto theta_rt =
            IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.c_str());
        auto react_chain_map = GetReactionChainMap(
            dagTasks_, tasksInfo_, sfOrder_, 1, chain,
            1);  // 1 and 1 are unused parameters in the function

        // add cause effective chain constraints together with objectives to
        // save time
        AddCauseEffectiveChainConstraintsFromReactMap(react_chain_map);

        LLint total_start_jobs =
            tasksInfo_.hyperPeriod / tasksInfo_.tasks[chain[0]].period;
        for (LLint start_instance_index = 0;
             start_instance_index < total_start_jobs; start_instance_index++) {
            JobCEC start_job = {chain[0], (start_instance_index)};
            JobCEC first_react_job = react_chain_map[start_job].back();
            model_.add(theta_rt >= (GetFinishTimeExpression(first_react_job) -
                                    GetStartTimeExpression(start_job)));
        }
        rtda_expression += theta_rt;
    }
    model_.add(IloMinimize(env_, rtda_expression));
    rtda_expression.end();
}
void SFOrderLPOptimizer::AddDataAgeObj() {
    IloExpr rtda_expression(env_);
    int chain_count = 0;
    for (auto chain : dagTasks_.chains_) {
        std::string var_name = "Chain_" + std::to_string(chain_count++) + "_DA";
        auto theta_da =
            IloNumVar(env_, 0, IloInfinity, IloNumVar::Float, var_name.c_str());
        auto da_chain_map = GetDataAgeChainMap(
            dagTasks_, tasksInfo_, sfOrder_, 1, chain,
            1);  // 1 and 1 are unused parameters in the function

        // add cause effective chain constraints together with objectives to
        // save time
        AddCauseEffectiveChainConstraintsFromDaMap(da_chain_map);

        LLint total_start_jobs =
            tasksInfo_.hyperPeriod / tasksInfo_.tasks[chain.back()].period;
        for (LLint start_instance_index = 0;
             start_instance_index < total_start_jobs; start_instance_index++) {
            JobCEC sink_job = {chain.back(), (start_instance_index)};
            JobCEC last_reading_job = da_chain_map[sink_job][0];
            model_.add(theta_da >= (GetFinishTimeExpression(sink_job) -
                                    GetStartTimeExpression(last_reading_job)));
        }
        rtda_expression += theta_da;
    }
    model_.add(IloMinimize(env_, rtda_expression));
    rtda_expression.end();
    if (GlobalVariablesDAGOpt::debugMode)
        WriteModelToFile("AddDataAgeObj.lp");
}
void SFOrderLPOptimizer::AddSensorFusionObj() { CoutError("Not implemented!"); }

IloExpr SFOrderLPOptimizer::GetStartTimeExpression(JobCEC &jobCEC) {
    IloExpr exp(env_);
    if (jobCEC.taskId < 0 || jobCEC.taskId >= tasksInfo_.N) {
        CoutError("GetStartTime receives invalid jobCEC!");
    }
    int jobNumInHyperPeriod =
        tasksInfo_.hyperPeriod / tasksInfo_.tasks[jobCEC.taskId].period;
    exp += varArray_[GetJobUniqueId(jobCEC, tasksInfo_)];
    if (jobCEC.jobId >= 0 || jobCEC.jobId % jobNumInHyperPeriod == 0)
        exp += jobCEC.jobId / jobNumInHyperPeriod * tasksInfo_.hyperPeriod;
    else
        exp +=
            (jobCEC.jobId / jobNumInHyperPeriod - 1) * tasksInfo_.hyperPeriod;
    return exp;
}

IloExpr SFOrderLPOptimizer::GetFinishTimeExpression(JobCEC &jobCEC) {
    return GetStartTimeExpression(jobCEC) +
           GetExecutionTime(jobCEC, tasksInfo_);
}

void SFOrderLPOptimizer::UpdateOptimizedStartTimeVector(
    IloNumArray &values_optimized) {
    for (int i = 0; i < numVariables_; i++) {
        optimizedStartTimeVector_(i) = values_optimized[i];
        // Round the optimized start time to interger when locate in 1e-4 range.
        // This can avoid float number precision error in further feasibility
        // check.
        if (abs(round(optimizedStartTimeVector_(i)) -
                optimizedStartTimeVector_(i)) < 1e-4) {
            optimizedStartTimeVector_(i) = round(optimizedStartTimeVector_(i));
        }
    }
}

}  // namespace OrderOptDAG_SPACE