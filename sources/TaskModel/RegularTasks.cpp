#include "RegularTasks.h"
namespace RegularTaskSystem
{
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
    TaskSet ReadTaskSet(string path, string priorityType, int taskSetType )
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

    void UpdateTaskSetExecutionTime(TaskSet &taskSet, VectorDynamic executionTimeVec, int lastTaskDoNotNeedOptimize)
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

}