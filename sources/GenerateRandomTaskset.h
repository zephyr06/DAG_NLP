#include <time.h>
#include <cmath>

#include <boost/program_options/options_description.hpp>

#include "RegularTasks.h"
#include "DAG_Model.h"
#include "Parameters.h"

using namespace RegularTaskSystem;
namespace po = boost::program_options;

// vector<int> PeriodSetAM = {1, 2, 5, 10, 20, 50, 100, 200, 1000};
vector<int> PeriodSetAM = {2, 5, 10, 20, 50, 100, 200};
// vector<int> PeriodSetAM = {100,200,300,400,500,600,800,1000,1200};

vector<double> Uunifast(int N, double utilAll)
{
    double sumU = utilAll;
    vector<double> utilVec(N, 0);

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

/**
 * @brief generate a random number within the range [a,b];
 * a must be smaller than b
 * 
 * @param a 
 * @param b 
 * @return double 
 */
double RandRange(double a, double b)
{
    if (b < a)
        CoutError("Range Error in RandRange");
    return a + (b - a) * double(rand()) / RAND_MAX;
}
/**
 * @brief 
 * 
 * @param N 
 * @param totalUtilization 
 * @param numberOfProcessor 
 * @param periodMin 
 * @param periodMax 
 * @param taskSetType 1 is normal, 2 is selecting periods from a given range
 * @param deadlineType  1 means random, 0 means implicit
 * @return TaskSet 
 */
TaskSet GenerateTaskSet(int N, double totalUtilization,
                        int numberOfProcessor, int periodMin,
                        int periodMax, int taskSetType = 1, int deadlineType = 0)
{
    vector<double> utilVec = Uunifast(N, totalUtilization);
    TaskSet tasks;
    int periodMaxRatio = periodMax / periodMin;

    for (int i = 0; i < N; i++)
    {

        int periodCurr = 0;
        if (taskSetType == 1)
        {
            periodCurr = (1 + rand() % periodMaxRatio) * periodMin;
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = RandRange(ceil(periodCurr * utilVec[i]), periodCurr);
            Task task(0, periodCurr,
                      0, ceil(periodCurr * utilVec[i]),
                      deadline, i,
                      rand() % numberOfProcessor);
            tasks.push_back(task);
        }

        else if (taskSetType == 2)
        {
            periodCurr = int(PeriodSetAM[rand() % PeriodSetAM.size()] * timeScaleFactor);
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = round(RandRange(ceil(periodCurr * utilVec[i]), periodCurr));
            Task task(0, periodCurr,
                      0, int(periodCurr * utilVec[i]),
                      deadline, i,
                      rand() % numberOfProcessor);
            tasks.push_back(task);
        }
        else
            CoutError("Not recognized taskSetType!");
    }
    return tasks;
}
void WriteTaskSets(ofstream &file, TaskSet &tasks)
{
    int N = tasks.size();
    file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId\n";
    for (int i = 0; i < N; i++)
    {
        file << tasks[i].id << "," << tasks[i].offset << ","
             << tasks[i].period << "," << tasks[i].overhead << ","
             << tasks[i].executionTime << "," << tasks[i].deadline
             << "," << tasks[i].processorId << "\n";
    }
}

using namespace DAG_SPACE;
DAG_Model GenerateDAG(int N, double totalUtilization,
                      int numberOfProcessor, int periodMin,
                      int periodMax, int taskSetType = 1, int deadlineType = 0)
{

    TaskSet tasks = GenerateTaskSet(N, totalUtilization,
                                    numberOfProcessor, periodMin, periodMax, taskSetType, deadlineType);
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
