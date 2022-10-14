#pragma once
#include "sources/Baseline/RTSS21IC/sources/Optimize.h"
#include "sources/Baseline/RTSS21IC/sources/GraphUtilsFromBGL.h"

#include "sources/Utils/OptimizeOrderUtils.h"

namespace DAG_SPACE
{
    RTSS21IC_NLP::RegularTaskSystem::TaskSet TaskSet2RTSSIC(const TaskSet &tasksInput)
    {
        RTSS21IC_NLP::RegularTaskSystem::TaskSet tasks;
        for (uint i = 0; i < tasksInput.size(); i++)
        {
            RegularTaskSystem::Task taskRef = tasksInput.at(i);
            RTSS21IC_NLP::RegularTaskSystem::Task task(taskRef.offset, taskRef.period, taskRef.overhead, taskRef.executionTime, taskRef.deadline, taskRef.id, taskRef.processorId);
            tasks.push_back(task);
        }
        return tasks;
    }

    RTSS21IC_NLP::DAG_SPACE::DAG_Model DAG_Model2RTSS21IC(DAG_Model &dagTasks)
    {
        RTSS21IC_NLP::RegularTaskSystem::TaskSet tasks = TaskSet2RTSSIC(dagTasks.tasks);

        RTSS21IC_NLP::DAG_SPACE::MAP_Prev mapPrev;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            size_t indexNext = itr->first;
            mapPrev[indexNext] = TaskSet2RTSSIC(tasksPrev);
        }

        RTSS21IC_NLP::DAG_SPACE::DAG_Model dag21(tasks, mapPrev);
        return dag21;
    }

    DAG_SPACE::ScheduleResult
    ScheduleRTSS21IC(DAG_SPACE::DAG_Model &dagTasks)
    {
        RTSS21IC_NLP::DAG_SPACE::DAG_Model dagTasksIC = DAG_Model2RTSS21IC(dagTasks);

        RTSS21IC_NLP::DAG_SPACE::OptimizeResult sth = RTSS21IC_NLP::DAG_SPACE::OptimizeScheduling(dagTasksIC);
        VectorDynamic stvAfterOpt = sth.optimizeVariable;
        if (debugMode == 1)
        {
            std::cout << "The result after optimization is " << RTSS21IC_NLP::Color::green << sth.optimizeError
                      //  << Color::blue << res
                      << RTSS21IC_NLP::Color::def << std::endl;
        }
        DAG_SPACE::ScheduleResult res;
        res.schedulable_ = sth.optimizeError < 1e-1;

        return res;
    }

}