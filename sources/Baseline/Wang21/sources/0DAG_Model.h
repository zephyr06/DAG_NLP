#pragma once

#include "sources/TaskModel/RegularTasks.h"
namespace RTSS21IC_NLP
{

    namespace DAG_SPACE
    {
        typedef std::map<int, RTSS21IC_NLP::RegularTaskSystem::TaskSet> MAP_Prev;
        using namespace RegularTaskSystem;
        class OrderOptDAG_SPACE::DAG_Model
        {
        public:
            TaskSet tasks;
            MAP_Prev mapPrev;
            int N;

            OrderOptDAG_SPACE::DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev) : tasks(tasks),
                                                                              mapPrev(mapPrev)
            {
                N = tasks.size();
            }

            void addEdge(int prevIndex, int nextIndex)
            {
                mapPrev[nextIndex].push_back(tasks[prevIndex]);
            }

            void addTask(Task &task)
            {
                tasks.push_back(task);
                N++;
            }

            void print()
            {
                for (auto &task : tasks)
                    task.print();
                for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
                {
                    for (int i = 0; i < (itr->second).size(); i++)
                        std::cout << "Edge: " << ((itr->second)[i].id) << "-->" << (itr->first) << endl;
                }
            }
            TaskSet GetTasks() const
            {
                return tasks;
            }

            int edgeNumber()
            {
                int count = 0;
                for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
                {
                    count += (itr->second).size();
                }
                return count;
            }
        };

        OrderOptDAG_SPACE::DAG_Model ReadDAG_Tasks(string path, string priorityType = "orig")
        {
            TaskSet tasks = ReadTaskSet(path, priorityType);
            // some default parameters in this function
            string delimiter = ",";
            string token;
            string line;
            size_t pos = 0;

            MAP_Prev mapPrev;

            fstream file;
            file.open(path, ios::in);
            if (file.is_open())
            {
                string line;
                while (getline(file, line))
                {
                    if (line[0] != '*')
                        continue;
                    line = line.substr(1, int(line.size()) - 1);
                    vector<int> dataInLine;
                    while ((pos = line.find(delimiter)) != string::npos)
                    {
                        token = line.substr(0, pos);
                        int temp = atoi(token.c_str());
                        dataInLine.push_back(temp);
                        line.erase(0, pos + delimiter.length());
                    }
                    dataInLine.push_back(atoi(line.c_str()));
                    mapPrev[dataInLine[1]].push_back(tasks[dataInLine[0]]);
                }

                OrderOptDAG_SPACE::DAG_Model ttt(tasks, mapPrev);

                if (debugMode == 1)
                    cout << "Finish reading the data file succesfully!\n";
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
    }

} // namespace RTSS21IC_NLP