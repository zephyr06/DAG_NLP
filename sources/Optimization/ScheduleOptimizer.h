#ifndef OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#define OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <unordered_map>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/OptimizeOrderUtils.h" // ScheduleResult
// #include "sources/Utils/Parameters.h"
// #include "sources/Factors/RTDA_Factor.h"
// #include "sources/Factors/SensorFusionFactor.h"
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
        void Optimize(DAG_Model &dagTasks, ScheduleResult &result);

        // function OptimizeObjWeighted() will optimize weighted objectives
        // TODO(Dong): the primary job of ScheduleOptimizer is taking a job order and return a startTimeVector, constructing a ScheduleResult struct is confusing for other users because they don't know how to construct it properly
        void OptimizeObjWeighted(DAG_Model &dagTasks, ScheduleResult &result);

        // TODO(Dong): similar as above, it should not return ScheduleResult unless there's a good reason
        ScheduleResult getOptimizedResult();

    protected:
        void AddVariables();

        void AddDBFConstraints();
        void AddDDLConstraints();
        void AddCauseEffectiveChainConstraints();
        void AddCauseEffectiveChainConstraintsFromReactMap(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map);
        void AddSensorFusionConstraints();
        void AddObjectives();
        void AddWeightedObjectives();
        IloExpr GetStartTimeExpression(JobCEC &jobCEC);
        IloExpr GetFinishTimeExpression(JobCEC &jobCEC);
        void UpdateOptimizedResult(IloNumArray &values_optimized);
        void setScheduleResult(ScheduleResult &res);
        inline void setDagTasks(DAG_Model &dagTasks);
        void setTasksInfo(TaskSetInfoDerived &info);

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