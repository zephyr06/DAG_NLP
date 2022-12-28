#ifndef SFORDER_LP_OPTIMIZER_H_
#define SFORDER_LP_OPTIMIZER_H_
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
    class SFOrderLPOptimizer
    {
    public:
        SFOrderLPOptimizer(const DAG_Model &dagTasks, SFOrder &sfOrder) : dagTasks_(dagTasks), sfOrder_(sfOrder)
        {
            env_.end();
            useWeightedObj_ = false;
            hasBeenInitialized_ = false;
        }

        void Init();
        void ClearCplexMemory();

        void Optimize(const std::vector<uint> &processorJobVec);

        // useWeightedObj_ will decide whether optimize max RTDAs or weighted objectives
        inline void setUseWeightedObjType(bool useWeightedObj = false)
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
        inline void setInitialStartTimeVector(const VectorDynamic &initialStartTimeVector)
        {
            initialStartTimeVector_ = initialStartTimeVector;
        }
        inline void setOptimizedStartTimeVector(const VectorDynamic &optimizedStartTimeVector)
        {
            optimizedStartTimeVector_ = optimizedStartTimeVector;
        }
        inline void setOptimizedStartTimeVector()
        {
            optimizedStartTimeVector_ = GenerateVectorDynamic(tasksInfo_.variableDimension);
        }
        inline void setProcessorJobVec(const std::vector<uint> &processorJobVec)
        {
            processorJobVec_ = processorJobVec;
        }
        inline void setTasksInfo(const TaskSetInfoDerived &info)
        {
            tasksInfo_ = info;
        }

    public:
        IloEnv env_;
        IloModel model_;
        IloCplex cplexSolver_;
        IloNumVarArray varArray_;
        VectorDynamic initialStartTimeVector_;
        VectorDynamic optimizedStartTimeVector_;
        std::vector<uint> processorJobVec_;
        TaskSetInfoDerived tasksInfo_;
        const DAG_Model &dagTasks_;
        SFOrder &sfOrder_;
        int numVariables_;
        bool useWeightedObj_;
        bool hasBeenInitialized_;
    };
} // namespace OrderOptDAG_SPACE
#endif // SFORDER_LP_OPTIMIZER_H_