#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <math.h>
// #include <filesystem>
#include <boost/integer/common_factor.hpp>
#include <boost/function.hpp>
// #include <CppUnitLite/TestHarness.h>

#include "colormod.h"

#include "Parameters.h"
#include "DeclareDAG.h"
#include "testMy.h"

namespace RTSS21IC_NLP
{

    using namespace std;

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
            std::string priorityType;

            // initializer

            Task() : offset(0), period(0),
                     overhead(0), executionTime(0.0),
                     deadline(0) { priorityType = priorityMode; }
            Task(int offset, int period, int overhead, double executionTime,
                 int deadline) : offset(offset), period(period),
                                 overhead(overhead), executionTime(executionTime),
                                 deadline(deadline)
            {
                id = -1;
                processorId = -1;
                priorityType = priorityMode;
            }
            Task(int offset, int period, int overhead, double executionTime,
                 int deadline, int id, int processorId) : offset(offset), period(period),
                                                          overhead(overhead), executionTime(executionTime),
                                                          deadline(deadline), id(id),
                                                          processorId(processorId) { priorityType = priorityMode; }
            double priority()
            {
                if (priorityType == "RM")
                {
                    if (period > 0)
                        return 1.0 / period;
                    else
                        CoutError("Period parameter less or equal to 0!");
                }
                else if (priorityType == "orig")
                    return id;
                else
                    CoutError("Priority settings not recognized!");
                return -1;
            }
            /**
             * only used in ReadTaskSet because the input parameter's type is int
             **/
            Task(std::vector<double> dataInLine)
            {
                if (dataInLine.size() != 7)
                {
                   std::cout << Color::red << "The length of dataInLine in Task constructor is wrong! Must be 7!\n"
                         << Color::def << std::endl;
                    throw;
                }
                id = dataInLine[0];
                offset = dataInLine[1];
                period = dataInLine[2];
                overhead = dataInLine[3];
                executionTime = dataInLine[4];
                deadline = dataInLine[5];
                processorId = dataInLine[6];
            }

            void print()
            {
               std::cout << "The period is: " << period << " The executionTime is " << executionTime << " The deadline is "
                     << deadline << " The overhead is " << overhead << " The offset is " << offset << std::endl;
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

        typedef std::vector<RTSS21IC_NLP::RegularTaskSystem::Task> TaskSet;

        void Print(TaskSet &tasks)
        {
           std::cout << "The task set is printed as follows" << std::endl;
            for (auto &task : tasks)
                task.print();
        }

        template <typename T>
       std::vector<T> GetParameter(const TaskSet &taskset, string parameterType)
        {
            uint N = taskset.size();
           std::vector<T> parameterList;
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
                   std::cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                         << Color::def << std::endl;
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
                   std::cout << Color::red << "parameterType in GetParameter is not recognized!\n"
                         << Color::def << std::endl;
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
        bool compareUtilization(Task task1, Task task2)
        {
            return task1.utilization() < task2.utilization();
        };

        double Utilization(const TaskSet &tasks)
        {
           std::vector<int> periodHigh = GetParameter<int>(tasks, "period");
           std::vector<double> executionTimeHigh = GetParameter<double>(tasks, "executionTime");
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
               std::cout << Color::red << "Empty task set in HyperPeriod()!\n";
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
                       std::cout << Color::red << "The hyper-period over flows!" << Color::def << std::endl;
                        throw;
                    }
                }
                return hyper;
            }
        }

        TaskSet Reorder(TaskSet tasks, string priorityType)
        {
            if (priorityType == "RM")
            {
                sort(tasks.begin(), tasks.end(), comparePeriod);
            }
            else if (priorityType == "utilization")
            {
                sort(tasks.begin(), tasks.end(), compareUtilization);
            }
            else if (priorityType == "orig")
            {
                ;
            }
            else
            {
               std::cout << Color::red << "Unrecognized priorityType in Reorder!\n"
                     << Color::def << std::endl;
                throw;
            }
            return tasks;
        }
        TaskSet ReadTaskSet(std::string path, string priorityType = "RM", int taskSetType = 1)
        {
            // some default parameters in this function
            string delimiter = ",";
            string token;
            string line;
            size_t pos = 0;

           std::vector<Task> taskSet;

            fstream file;
            file.open(path, ios::in);
            if (file.is_open())
            {
                string line;
                while (getline(file, line))
                {
                    if (!(line[0] >= '0' && line[0] <= '9'))
                        continue;
                   std::vector<double> dataInLine;
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
                    taskSet.push_back(taskCurr);
                }

                TaskSet ttt(taskSet);
                ttt = Reorder(ttt, priorityType);
                if (debugMode == 1)
                   std::cout << "Finish reading the data file succesfully!\n";
                return ttt;
            }
            else
            {
               std::cout << Color::red << "The path does not exist in ReadTaskSet!" << std::endl
                     << path
                     << Color::def << std::endl;
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
         * @return ProcessorTaskSet map type! processorId tostd::vector<task ID>
         */
        ProcessorTaskSet ExtractProcessorTaskSet(TaskSet &tasks)
        {
            int N = tasks.size();
            ProcessorTaskSet processorTasks;
            for (int i = 0; i < N; i++)
            {
                if (processorTasks.find(tasks[i].processorId) == processorTasks.end())
                {
                   std::vector<int> ttt{tasks[i].id};
                    processorTasks[tasks[i].processorId] = ttt;
                }
                else
                {
                    processorTasks[tasks[i].processorId].push_back(tasks[i].id);
                }
            }
            return processorTasks;
        }
    }
} // namespace RTSS21IC_NLP
