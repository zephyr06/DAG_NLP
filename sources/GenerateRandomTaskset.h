#include <time.h>
#include <cmath>

#include <boost/program_options/options_description.hpp>

#include "RegularTasks.h"
#include "DAG_Model.h"
#include "Parameters.h"

using namespace RegularTaskSystem;
namespace po = boost::program_options;

vector<double> Uunifast(int N, double utilAll)
{
    double sumU = utilAll;
    vector<double> utilVec(N, 0);
    srand(time(0));
    double nextU;
    for (size_t i = 1; i < N; i++)
    {
        nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
        utilVec[i - 1] = sumU - nextU;
        sumU = nextU;
    }
    utilVec[N - 1] = nextU;
    return utilVec;
}

TaskSet GenerateTaskSet(int N, double totalUtilization,
                        int numberOfProcessor, int periodMin,
                        int periodMax)
{
    vector<double> utilVec = Uunifast(N, totalUtilization);
    TaskSet tasks;
    int periodMaxRatio = periodMax / periodMin;
    for (int i = 0; i < N; i++)
    {
        int periodCurr = rand() % periodMaxRatio * periodMin;
        Task task(0, periodCurr,
                  0, round(periodCurr * utilVec[i]),
                  periodCurr, i,
                  rand() % numberOfProcessor);
        tasks.push_back(task);
    }
    return tasks;
}
void WriteTaskSets(ofstream &file, TaskSet &tasks)
{
    int N = tasks.size();
    file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine\n";
    for (int i = 0; i < N; i++)
    {
        file << tasks[i].id << "," << tasks[i].offset << ","
             << tasks[i].period << "," << tasks[i].overhead << ","
             << tasks[i].executionTime << "," << tasks[i].deadline << "\n";
    }
}

using namespace DAG_SPACE;
DAG_Model GenerateDAG(int N, double totalUtilization,
                      int numberOfProcessor, int periodMin,
                      int periodMax)
{

    TaskSet tasks = GenerateTaskSet(N, totalUtilization,
                                    numberOfProcessor, periodMin, periodMax);
    MAP_Prev mapPrev;
    DAG_Model dagModel(tasks, mapPrev);
    // add edges randomly
    for (int i = 0; i < N; i++)
    {
        for (int j = i + 1; j < N; j++)
        {
            if (double(rand()) / RAND_MAX < parallelFactor)
            {
                dagModel.addEdge(i, j);
            }
        }
    }
    return dagModel;
}

void WriteDAG(ofstream &file, DAG_Model &tasksDAG)
{
    WriteTaskSets(file, tasksDAG.tasks);
    for (auto itr = tasksDAG.mapPrev.begin(); itr != tasksDAG.mapPrev.end(); itr++)
    {
        for (int i = 0; i < itr->second.size(); i++)
            file << "*" << (itr->first) << "," << ((itr->second)[i].id) << "\n";
    }
}