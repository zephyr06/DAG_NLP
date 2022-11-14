#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
// #include <filesystem>
#include <boost/integer/common_factor.hpp>
#include <boost/function.hpp>
#include <CppUnitLite/TestHarness.h>

#include "sources/Tools/colormod.h"

#include "sources/Utils/Parameters.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Tools/testMy.h"
using namespace std;

typedef std::map<int, std::vector<int>> ProcessorTaskSet;

bool CompareStringNoCase(const std::string &s1, const std::string s2)
{
    return strcasecmp(s1.c_str(), s2.c_str()) == 0;
}

namespace RegularTaskSystem
{
    class Task
    {
    public:
        // Member list
        int offset;
        int period;
        int overhead;
        double executionTime;
        int deadline;
        int id;
        int processorId;
        int coreRequire;
        int taskType;
        int priority_; // its value needs to be assigned by other functions before usage
        std::string priorityType_;

        // initializer

        Task() : offset(0), period(0),
                 overhead(0), executionTime(0.0),
                 deadline(0), coreRequire(1), id(-1), taskType(0) {}
        Task(int offset, int period, int overhead, double executionTime,
             int deadline) : offset(offset), period(period),
                             overhead(overhead), executionTime(executionTime),
                             deadline(deadline), taskType(0)
        {
            id = -1;
            processorId = -1;
            coreRequire = 1;
            priorityType_ = priorityMode;
        }
        Task(int offset, int period, int overhead, double executionTime,
             int deadline, int id, int processorId) : offset(offset), period(period),
                                                      overhead(overhead),
                                                      executionTime(executionTime),
                                                      deadline(deadline), id(id),
                                                      processorId(processorId), taskType(0)
        {
            coreRequire = 1;
            priorityType_ = priorityMode;
        }
        Task(int offset, int period, int overhead, double executionTime,
             int deadline, int id, int processorId,
             int coreRequire) : offset(offset), period(period),
                                overhead(overhead), executionTime(executionTime),
                                deadline(deadline), id(id),
                                processorId(processorId),
                                coreRequire(coreRequire), taskType(0) { priorityType_ = priorityMode; }
        Task(int offset, int period, int overhead, double executionTime,
             int deadline, int id, int processorId,
             int coreRequire, int taskType) : offset(offset), period(period),
                                              overhead(overhead), executionTime(executionTime),
                                              deadline(deadline), id(id),
                                              processorId(processorId),
                                              coreRequire(coreRequire),
                                              taskType(taskType) { priorityType_ = priorityMode; }

        // modify public member priorityType_ to change how to calculate the value: priority_
        double priority()
        {
            if (CompareStringNoCase(priorityType_, "RM"))
            {
                if (period > 0)
                    return 1.0 / period;
                else
                    CoutError("Period parameter less or equal to 0!");
            }
            else if (CompareStringNoCase(priorityType_, "orig"))
                return id;
            else if (CompareStringNoCase(priorityType_, "assigned"))
                return priority_;
            else
                CoutError("Priority settings not recognized!");
            return -1;
        }
        /**
         * only used in ReadTaskSet because the input parameter's type is int
         **/
        Task(vector<double> dataInLine)
        {
            if (dataInLine.size() != 8)
            {
                // cout << Color::red << "The length of dataInLine in Task constructor is wrong! Must be 8!\n"
                //      << Color::def << endl;
                // throw;
            }
            id = dataInLine[0];
            offset = dataInLine[1];
            period = dataInLine[2];
            overhead = dataInLine[3];
            executionTime = dataInLine[4];
            deadline = dataInLine[5];
            processorId = dataInLine[6];
            coreRequire = dataInLine[7];
            taskType = 0;
            if (dataInLine.size() > 8)
                taskType = dataInLine[8];
            if (coreRequire < 1)
                coreRequire = 1;
        }

        void print()
        {
            cout << "The period is: " << period << " The executionTime is " << executionTime << " The deadline is "
                 << deadline << " The overhead is " << overhead << " The offset is " << offset
                 << " The coreRequire is " << coreRequire
                 << " The taskType is " << taskType << endl;
        }

        double utilization() const
        {
            return double(executionTime) / period;
        }

        int slack()
        {
            return deadline - executionTime;
        }
    };

    typedef std::vector<RegularTaskSystem::Task> TaskSet;
    void Print(TaskSet &tasks)
    {
        cout << "The task set is printed as follows" << endl;
        for (auto &task : tasks)
            task.print();
    }

    template <typename T>
    vector<T> GetParameter(const TaskSet &taskset, string parameterType)
    {
        uint N = taskset.size();
        vector<T> parameterList;
        parameterList.reserve(N);

        for (uint i = 0; i < N; i++)
        {
            if (parameterType == "period")
                parameterList.push_back((T)(taskset[i].period));
            else if (parameterType == "executionTime")
                parameterList.push_back((T)(taskset[i].executionTime));
            else if (parameterType == "overhead")
                parameterList.push_back((T)(taskset[i].overhead));
            else if (parameterType == "deadline")
                parameterList.push_back((T)(taskset[i].deadline));
            else if (parameterType == "offset")
                parameterList.push_back((T)(taskset[i].offset));
            else
            {
                cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                     << Color::def << endl;
                throw;
            }
        }
        return parameterList;
    }
    template <typename T>
    VectorDynamic GetParameterVD(const TaskSet &taskset, string parameterType)
    {
        uint N = taskset.size();
        VectorDynamic parameterList;
        parameterList.resize(N, 1);
        parameterList.setZero();

        for (uint i = 0; i < N; i++)
        {
            if (parameterType == "period")
                parameterList(i, 0) = ((T)(taskset[i].period));
            else if (parameterType == "executionTime")
                parameterList(i, 0) = ((T)(taskset[i].executionTime));
            else if (parameterType == "overhead")
                parameterList(i, 0) = ((T)(taskset[i].overhead));
            else if (parameterType == "deadline")
                parameterList(i, 0) = ((T)(taskset[i].deadline));
            else if (parameterType == "offset")
                parameterList(i, 0) = ((T)(taskset[i].offset));
            else
            {
                cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                     << Color::def << endl;
                throw;
            }
        }
        return parameterList;
    }

    // some helper function for Reorder
    static bool comparePeriod(Task task1, Task task2)
    {
        return task1.period < task2.period;
    };
    static bool compareDeadline(Task task1, Task task2)
    {
        return task1.deadline < task2.deadline;
    };

    bool compareUtilization(Task task1, Task task2)
    {
        return task1.utilization() < task2.utilization();
    };

    double Utilization(const TaskSet &tasks)
    {
        vector<int> periodHigh = GetParameter<int>(tasks, "period");
        vector<double> executionTimeHigh = GetParameter<double>(tasks, "executionTime");
        int N = periodHigh.size();
        double utilization = 0;
        for (int i = 0; i < N; i++)
            utilization += double(executionTimeHigh[i]) / periodHigh[i];
        return utilization;
    }

    // Recursive function to return gcd of a and b
    long long gcd(long long int a, long long int b)
    {
        if (b == 0)
            return a;
        return gcd(b, a % b);
    }

    // Function to return LCM of two numbers
    long long int lcm(long long int a, long long int b)
    {
        return (a / gcd(a, b)) * b;
    }

    long long int HyperPeriod(const TaskSet &tasks)
    {
        int N = tasks.size();
        if (N == 0)
        {
            cout << Color::red << "Empty task set in HyperPeriod()!\n";
            throw;
        }
        else
        {
            long long int hyper = tasks[0].period;
            for (int i = 1; i < N; i++)
            {
                hyper = lcm(hyper, tasks[i].period);
                if (hyper < 0 || hyper > LLONG_MAX)
                {
                    cout << Color::red << "The hyper-period over flows!" << Color::def << endl;
                    throw;
                }
            }
            return hyper;
        }
    }

    // should not be used anymore
    TaskSet Reorder(TaskSet tasks, string priorityType)
    {
        if (CompareStringNoCase(priorityType, "RM"))
        {
            sort(tasks.begin(), tasks.end(), comparePeriod);
        }
        else if (CompareStringNoCase(priorityType, "utilization"))
        {
            sort(tasks.begin(), tasks.end(), compareUtilization);
        }
        else if (CompareStringNoCase(priorityType, "DM"))
        {
            sort(tasks.begin(), tasks.end(), compareDeadline);
        }
        else if (CompareStringNoCase(priorityType, "orig"))
        {
            ;
        }
        else
        {
            cout << Color::red << "Unrecognized priorityType in Reorder!\n"
                 << Color::def << endl;
            throw;
        }
        return tasks;
    }
    TaskSet ReadTaskSet(string path, string priorityType = "RM", int taskSetType = 1)
    {
        // some default parameters in this function
        string delimiter = ",";
        string token;
        string line;
        size_t pos = 0;

        vector<Task> taskSet;

        fstream file;
        file.open(path, ios::in);
        if (file.is_open())
        {
            string line;
            while (getline(file, line))
            {
                if (!(line[0] >= '0' && line[0] <= '9'))
                    continue;
                vector<double> dataInLine;
                while ((pos = line.find(delimiter)) != string::npos)
                {
                    token = line.substr(0, pos);
                    double temp = atof(token.c_str());
                    dataInLine.push_back(temp);
                    line.erase(0, pos + delimiter.length());
                }
                dataInLine.push_back(atoi(line.c_str()));
                // dataInLine.erase(dataInLine.begin());
                Task taskCurr(dataInLine);
                taskCurr.priorityType_ = priorityType;
                taskSet.push_back(taskCurr);
            }

            TaskSet ttt(taskSet);
            // ttt = Reorder(ttt, priorityType);
            return ttt;
        }
        else
        {
            cout << Color::red << "The path does not exist in ReadTaskSet!" << endl
                 << path
                 << Color::def << endl;
            throw;
        }
    }

    void UpdateTaskSetExecutionTime(TaskSet &taskSet, VectorDynamic executionTimeVec, int lastTaskDoNotNeedOptimize = -1)
    {
        int N = taskSet.size();

        for (int i = lastTaskDoNotNeedOptimize + 1; i < N; i++)
            taskSet[i].executionTime = executionTimeVec(i - lastTaskDoNotNeedOptimize - 1, 0);
    }
    /**
     * @brief
     *
     * @param tasks
     * @return ProcessorTaskSet map type! processorId to vector<task ID>
     */
    ProcessorTaskSet ExtractProcessorTaskSet(const TaskSet &tasks)
    {
        int N = tasks.size();
        ProcessorTaskSet processorTasks;
        for (int i = 0; i < N; i++)
        {
            if (processorTasks.find(tasks.at(i).processorId) == processorTasks.end())
            {
                vector<int> ttt{tasks.at(i).id};
                processorTasks[tasks.at(i).processorId] = ttt;
            }
            else
            {
                processorTasks[tasks.at(i).processorId].push_back(tasks.at(i).id);
            }
        }
        return processorTasks;
    }

    class TaskSetInfoDerived
    {
    public:
        TaskSet tasks;
        int N;
        LLint hyperPeriod;
        LLint variableDimension;
        vector<LLint> sizeOfVariables;
        LLint length;
        ProcessorTaskSet processorTaskSet;

        TaskSetInfoDerived() {}

        TaskSetInfoDerived(const TaskSet &tasksInput)
        {
            tasks = tasksInput;
            N = tasks.size();
            hyperPeriod = HyperPeriod(tasks);
            variableDimension = 0;
            length = 0;
            for (int i = 0; i < N; i++)
            {
                LLint size = hyperPeriod / tasks[i].period;
                sizeOfVariables.push_back(size);
                variableDimension += size;
                length += sizeOfVariables[i];
            }
            processorTaskSet = ExtractProcessorTaskSet(tasks);
        }
    };

}