#include <time.h>
#include <cmath>

#include <boost/program_options/options_description.hpp>

#include "sources/TaskModel/RegularTasks.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/Parameters.h"

using namespace RegularTaskSystem;
namespace po = boost::program_options;

// vector<int> PeriodSetAM = {1, 2, 5, 10, 20, 50, 100, 200, 1000};
// vector<int> PeriodSetAM = {2, 5, 10, 20, 50, 100, 200};
vector<int> PeriodSetAM = {100, 200, 300};
// vector<int> PeriodSetAM = {100, 200, 300, 400, 500, 600, 800, 1000, 1200};
const vector<int> PeriodSetWaters = {1, 2, 5, 10, 20, 50, 100, 200, 1000};
const vector<int> PeriodPDFWaters = {3, 2, 2, 25, 25, 3, 20, 1, 4};
const vector<int> PeriodCDFWaters = {3, 5, 7, 32, 57, 60, 80, 81, 85};

std::vector<double> Uunifast(int N, double utilAll, bool boundU = true)
    {
        double sumU = utilAll;
        std::vector<double> utilVec(N, 0);

        double nextU;
        for (int i = 1; i < N; i++)
        {

            nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
            if (boundU)
            {
                utilVec[i - 1] = std::min(1.0, sumU - nextU);
                nextU += std::max(0.0, sumU - nextU - 1.0);
            }
            else
            {
                utilVec[i - 1] = sumU - nextU;
            }
            sumU = nextU;
        }
        utilVec[N - 1] = nextU;
        return utilVec;
    }

TaskSet GenerateTaskSet(int N, double totalUtilization,
                        int numberOfProcessor, int periodMin,
                        int periodMax, int coreRequireMax,
                        int taskSetType = 1, int deadlineType = 0)
{
    vector<double> utilVec = Uunifast(N, totalUtilization);
    TaskSet tasks;
    int periodMaxRatio = periodMax / periodMin;

    for (int i = 0; i < N; i++)
    {

        int periodCurr = 0;
        int processorId = rand() % numberOfProcessor;
        int coreRequire = 1 + rand() % (coreRequireMax);
        if (taskSetType == 1)
        {
            periodCurr = (1 + rand() % periodMaxRatio) * periodMin;
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = RandRange(ceil(periodCurr * utilVec[i]), periodCurr);
            Task task(0, periodCurr,
                      0, std::max(1.0, ceil(periodCurr * utilVec[i])),
                      deadline, i,
                      processorId, coreRequire);
            tasks.push_back(task);
        }
        else if (taskSetType == 2)
        {
            periodCurr = int(PeriodSetAM[rand() % PeriodSetAM.size()] * timeScaleFactor);
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = round(RandRange(std::max(1.0, ceil(periodCurr * utilVec[i])), periodCurr));
            Task task(0, periodCurr,
                      0, std::max(1.0, ceil(periodCurr * utilVec[i])),
                      deadline, i,
                      processorId, coreRequire);
            tasks.push_back(task);
        }
        else if (taskSetType == 3)
        {
            int probability = rand() % PeriodCDFWaters.back();
            int period_idx = 0;
            while (probability > PeriodCDFWaters[period_idx])
            {
                period_idx++;
            }
            periodCurr = int(PeriodSetWaters[period_idx] * timeScaleFactor);
            double deadline = periodCurr;
            if (deadlineType == 1)
                deadline = round(RandRange(std::max(1.0, ceil(periodCurr * utilVec[i])), periodCurr));
            Task task(0, periodCurr,
                      0, std::max(1.0, ceil(periodCurr * utilVec[i])),
                      deadline, i,
                      processorId, coreRequire);
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
    file << "JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId,coreRequire\n";
    for (int i = 0; i < N; i++)
    {
        file << tasks[i].id << "," << tasks[i].offset << ","
             << tasks[i].period << "," << tasks[i].overhead << ","
             << tasks[i].executionTime << "," << tasks[i].deadline
             << "," << tasks[i].processorId
             << "," << tasks[i].coreRequire << "\n";
    }
}

using namespace OrderOptDAG_SPACE;
DAG_Model GenerateDAG(int N, double totalUtilization,
                      int numberOfProcessor, int periodMin,
                      int periodMax, int coreRequireMax,
                      int taskSetType = 1, int deadlineType = 0)
{

    TaskSet tasks = GenerateTaskSet(N, totalUtilization,
                                    numberOfProcessor, periodMin, periodMax, coreRequireMax, taskSetType, deadlineType);
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
        for (uint i = 0; i < itr->second.size(); i++)
            file << "*" << (itr->first) << "," << ((itr->second)[i].id) << "\n";
    }
}