#pragma once

#include "unordered_set"
#include "unordered_map"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"

namespace OrderOptDAG_SPACE
{

    struct TimeInstance
    {
        char type; // 's' or 'f'
        JobCEC job;
        double time;

        double getTime()
        {
            return time;
        }
        static bool compare(const TimeInstance &i1, const TimeInstance &i2)
        {
            return (i1.time < i2.time);
        }
    };

    class SFOrder
    {
    private:
        TaskSetInfoDerived tasksInfo_;
        struct SFPair
        {
            LLint startInstanceIndex;
            LLint finishInstanceIndex;
        };
        std::unordered_map<JobCEC, SFPair> jobSFMap_;

    public:
        std::vector<TimeInstance> instanceOrder_;
        SFOrder() {}

        // O(n log(n))
        SFOrder(TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo)
        {
            instanceOrder_.reserve(tasksInfo.length * 2);
            instanceIndexMap_.reserve(tasksInfo.length * 2);
            jobSFMap_.reserve(tasksInfo.length);
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                {
                    JobCEC job(i, j);
                    instanceOrder_.push_back(TimeInstance('s', job, GetStartTime(jobc, startTimeVector, tasksInfo)));
                    instanceOrder_.push_back(TimeInstance('f', job, GetFinishTime(jobc, startTimeVector, tasksInfo)));
                }
            }
            std::sort(instanceOrder_.begin(), instanceOrder_.end(), TimeInstance::compare);

            EstablishJobSFMap();
        }

        // O(n^2), could be improved to be O(n)
        void EstablishJobSFMap()
        {
            jobSFMap_.reserve(tasksInfo.length);
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                {
                    JobCEC job(i, j);
                    SFPair sfPair;
                    for (LLint i = 0; i < tasksInfo.length * 2; i++)
                    {
                        TimeInstance &inst = instanceOrder_[i];
                        if (inst.job == job)
                        {
                            if (inst.type == 's')
                                sfPair.startInstanceIndex = i;
                            else if (inst.type == 'f')
                            {
                                sfPair.finishInstanceIndex = i;
                                jobSFMap_[job] = sfPair;
                                break;
                            }
                        }
                    }
                }
            }
        }

        LLint GetJobStartInstancePosition(JobCEC &job)
        {
            return jobSFMap_[job].startInstanceIndex;
        }
        LLint GetJobFinishInstancePosition(JobCEC &job)
        {
            return jobSFMap_[job].finishInstanceIndex;
        }

        JobCEC RemoveJob(JobCEC job);

        void InsertStart(JobCEC job, LLint position);

        void InsertFinish(JobCEC, LLint position);
    };
} // namespace OrderOptDAG_SPACE
