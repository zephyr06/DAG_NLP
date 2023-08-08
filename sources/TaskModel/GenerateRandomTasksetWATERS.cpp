

#include "sources/TaskModel/GenerateRandomTasksetWATERS.h"

void AddChains2DAG(DAG_Model &dag_tasks, int numCauseEffectChain) {
  RandomChainsGenerator generator(dag_tasks);
  dag_tasks.chains_ = generator.GenerateChain(numCauseEffectChain);
}

DAG_Model GenerateDAG_WATERS15(int N, double totalUtilization, int numberOfProcessor, int periodMin,
                               int periodMax, int coreRequireMax, int sf_fork_num, int fork_sensor_num_min,
                               int fork_sensor_num_max, int numCauseEffectChain, int taskSetType,
                               int deadlineType) {
  TaskSet tasks = GenerateTaskSet(N, totalUtilization, numberOfProcessor, periodMin, periodMax,
                                  coreRequireMax, taskSetType, deadlineType);
  DAG_Model dagModel;
  dagModel.tasks = tasks;
  AddEdges2DAG_He21(dagModel, N);

  AddChains2DAG(dagModel, numCauseEffectChain);
  dagModel.sf_forks_ = dagModel.GetRandomForks(sf_fork_num, fork_sensor_num_min, fork_sensor_num_max);
  return dagModel;
}
