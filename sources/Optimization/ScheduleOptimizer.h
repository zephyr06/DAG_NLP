#ifndef OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
#define OPTIMIZATION_SCHEDULE_OPTIMIZER_H_
// #include <iostream>
// #include <memory>
// #include <vector>
// #include <unordered_map>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/OptimizeOrderUtils.h"
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
        ScheduleOptimizer(const DAG_Model &dagTasks) : dagTasks_(dagTasks)
        {
            env_.end();
            useWeightedObj_ = false;
        }

        void Optimize(const VectorDynamic &initialStartTimeVector, const std::vector<uint> &processorJobVec);

        void OptimizeWithJobOrder(const VectorDynamic &initialStartTimeVector, const std::vector<uint> &processorJobVec, const SFOrder &jobOrder);

        // useWeightedObj_ will decide wheterh optimize RTDA or weighted objectives
        inline void setObjType(bool useWeightedObj = false)
        {
            useWeightedObj_ = useWeightedObj;
        }

        VectorDynamic getOptimizedStartTimeVector();

        // protected:
        void AddVariables();
        void AddDBFConstraints();
        void AddDDLConstraints();
        void AddJobOrderConstraints(const SFOrder &jobOrder);
        void AddCauseEffectiveChainConstraints();
        void AddCauseEffectiveChainConstraintsFromReactMap(const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map);
        void AddSensorFusionConstraints();
        void AddObjectives();
        void AddNormalObjectives();   // RTDA obj
        void AddWeightedObjectives(); // weighted obj
        IloExpr GetStartTimeExpression(JobCEC &jobCEC);
        IloExpr GetFinishTimeExpression(JobCEC &jobCEC);
        void UpdateOptimizedStartTimeVector(IloNumArray &values_optimized);
        inline void setInitialStartTimeVector(const VectorDynamic &initialStartTimeVector);
        inline void setOptimizedStartTimeVector(const VectorDynamic &optimizedStartTimeVector);
        inline void setProcessorJobVec(const std::vector<uint> &processorJobVec);
        inline void setTasksInfo(const TaskSetInfoDerived &info);

    private:
        IloEnv env_;
        IloModel model_;
        IloCplex cplexSolver_;
        IloNumVarArray varArray_;
        VectorDynamic initialStartTimeVector_;
        VectorDynamic optimizedStartTimeVector_;
        std::vector<uint> processorJobVec_;
        TaskSetInfoDerived tasksInfo_;
        const DAG_Model &dagTasks_;
        int numVariables_;
        bool useWeightedObj_;
    };
} // namespace OrderOptDAG_SPACE
#endif // OPTIMIZATION_SCHEDULE_OPTIMIZER_H_