#ifndef SFORDER_LP_OPTIMIZER_H_
#define SFORDER_LP_OPTIMIZER_H_
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"

namespace OrderOptDAG_SPACE
{
  class SFOrderLPOptimizer
  {
  public:
    SFOrderLPOptimizer(const DAG_Model &dagTasks, SFOrder &sfOrder, int processorNum)
        : dagTasks_(dagTasks), sfOrder_(sfOrder), processorNum_(processorNum)
    {
      env_.end();
      hasBeenInitialized_ = false;
    }

    void Init();
    void ClearCplexMemory();

    void Optimize(const std::vector<uint> &processorJobVec);

    VectorDynamic getOptimizedStartTimeVector();

    // protected:
    void AddVariables();
    void AddDBFConstraints();
    void AddDDLConstraints();
    void AddJobOrderConstraints(const SFOrder &jobOrder);
    void AddCauseEffectiveChainConstraints();
    void AddCauseEffectiveChainConstraintsFromReactMap(
        const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map);
    void AddSensorFusionConstraints();
    void AddObjectives();
    void AddNormalObjectives(); // RTDA obj
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
    inline void setTasksInfo(const TaskSetInfoDerived &info) { tasksInfo_ = info; }

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
    bool hasBeenInitialized_;
    int processorNum_;
  };
} // namespace OrderOptDAG_SPACE
#endif // SFORDER_LP_OPTIMIZER_H_