#include "sources/TaskModel/GenerateRandomTaskset.h"

std::vector<double> Uunifast(int N, double utilAll, bool boundU) {
  double sumU = utilAll;
  std::vector<double> utilVec(N, 0);

  double nextU = 0;
  for (int i = 1; i < N; i++) {
    nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
    if (boundU) {
      utilVec[i - 1] = std::min(1.0, sumU - nextU);
      nextU += std::max(0.0, sumU - nextU - 1.0);
    } else {
      utilVec[i - 1] = sumU - nextU;
    }
    sumU = nextU;
  }
  utilVec[N - 1] = nextU;
  return utilVec;
}

TaskSet GenerateTaskSet(int N, double totalUtilization, int numberOfProcessor, int periodMin, int periodMax,
                        int coreRequireMax, int taskSetType, int deadlineType) {
  std::vector<double> utilVec = Uunifast(N, totalUtilization);
  TaskSet tasks;
  int periodMaxRatio = periodMax / periodMin;

  for (int i = 0; i < N; i++) {
    int periodCurr = 0;
    int processorId = rand() % numberOfProcessor;
    int coreRequire = 1 + rand() % (coreRequireMax);
    if (taskSetType == 1) // fully random task set
    {
      periodCurr = (1 + rand() % periodMaxRatio) * periodMin;
      double deadline = periodCurr;
      if (deadlineType == 1)
        deadline = RandRange(ceil(periodCurr * utilVec[i]), periodCurr);
      Task task(0, periodCurr, 0, std::max(1.0, ceil(periodCurr * utilVec[i])), deadline, i, processorId,
                coreRequire);
      tasks.push_back(task);
    } else if (taskSetType == 2) // random choice from AutoMobile set 'PeriodSetAM'
    {
      periodCurr = int(PeriodSetAM[rand() % PeriodSetAM.size()] * GlobalVariablesDAGOpt::timeScaleFactor);
      double deadline = periodCurr;
      if (deadlineType == 1)
        deadline = round(RandRange(std::max(1.0, ceil(periodCurr * utilVec[i])), periodCurr));
      Task task(0, periodCurr, 0, std::max(1.0, ceil(periodCurr * utilVec[i])), deadline, i, processorId,
                coreRequire);
      tasks.push_back(task);
    } else if (taskSetType == 3) // automobile periods with WATERS distribution
    {
      int probability = rand() % PeriodCDFWaters.back();
      int period_idx = 0;
      while (probability > PeriodCDFWaters[period_idx]) {
        period_idx++;
      }
      periodCurr = int(PeriodSetWaters[period_idx] * GlobalVariablesDAGOpt::timeScaleFactor);
      double deadline = periodCurr;
      if (deadlineType == 1)
        deadline = round(RandRange(std::max(1.0, ceil(periodCurr * utilVec[i])), periodCurr));
      Task task(0, periodCurr, 0, std::max(1.0, ceil(periodCurr * utilVec[i])), deadline, i, processorId,
                coreRequire);
      tasks.push_back(task);
    } else
      CoutError("Not recognized taskSetType!");
  }
  return tasks;
}

DAG_Model AddEdges2DAG_He21(DAG_Model &dag_tasks, int N) {
  MAP_Prev mapPrev;
  // add edges randomly
  for (int i = 0; i < N; i++) {
    for (int j = i + 1; j < N; j++) {
      if (double(rand()) / RAND_MAX < GlobalVariablesDAGOpt::parallelFactor) {
        dag_tasks.addEdge(i, j);
      }
    }
  }
  dag_tasks.ConstructBGL_Graph();
  return dag_tasks;
}

using namespace OrderOptDAG_SPACE;
DAG_Model GenerateDAG_He21(int N, double totalUtilization, int numberOfProcessor, int periodMin,
                           int periodMax, int coreRequireMax, int sf_fork_num, int fork_sensor_num_min,
                           int fork_sensor_num_max, int numCauseEffectChain, int taskSetType,
                           int deadlineType) {
  TaskSet tasks = GenerateTaskSet(N, totalUtilization, numberOfProcessor, periodMin, periodMax,
                                  coreRequireMax, taskSetType, deadlineType);

  DAG_Model dagModel;
  dagModel.tasks = tasks;
  AddEdges2DAG_He21(dagModel, N);

  return DAG_Model(tasks, dagModel.mapPrev, sf_fork_num, fork_sensor_num_min, fork_sensor_num_max,
                   numCauseEffectChain);
}

void WriteTaskSets(std::ofstream &file, const TaskSet &tasks) {
  int N = tasks.size();
  file << "JobID,Period,ExecutionTime,DeadLine,processorId"
          "\n";
  for (int i = 0; i < N; i++) {
    file << tasks[i].id
         // << "," << tasks[i].offset
         << ","
         << tasks[i].period
         //  << "," << tasks[i].overhead
         << "," << tasks[i].executionTime << "," << tasks[i].deadline << ","
         << tasks[i].processorId

         // << "," << tasks[i].coreRequire
         << "\n";
  }
}
void WriteDAG(std::ofstream &file, DAG_Model &dag_tasks) {
  // WriteTaskSets(file, dag_tasks.tasks);
  const TaskSet &tasks = dag_tasks.tasks;
  int N = dag_tasks.tasks.size();
  file << "JobID,Period,ExecutionTime,DeadLine,processorId"
          "\n";
  for (int i = 0; i < N; i++) {
    file << tasks[i].id
         // << "," << tasks[i].offset
         << ","
         << tasks[i].period
         //  << "," << tasks[i].overhead
         << "," << tasks[i].executionTime << "," << tasks[i].deadline << ","
         << tasks[i].processorId

         // << "," << tasks[i].coreRequire
         << "\n";
  }

  file << "Note: "
       << "*a,b"
       << " means a->b, i.e., a writes data and b reads data from it\n";
  for (const auto &chain : dag_tasks.chains_) {
    file << "@Chain:";
    for (int x : chain)
      file << x << ", ";
    file << "\n";
  }
  for (const auto &sf_fork : dag_tasks.sf_forks_) {
    file << "@Fork_source:";
    for (int x : sf_fork.source)
      file << x << ", ";
    file << "\n";
    file << "@Fork_sink:" << sf_fork.sink << ", \n";
  }
  for (auto itr = dag_tasks.mapPrev.begin(); itr != dag_tasks.mapPrev.end(); itr++) {
    for (uint i = 0; i < itr->second.size(); i++)
      file << "*" << ((itr->second)[i].id) << "," << (itr->first) << "\n";
  }
}