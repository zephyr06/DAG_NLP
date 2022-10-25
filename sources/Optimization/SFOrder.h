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
        double time;
        JobCEC job;

        double getTime()
        {
            return time;
        }
        static bool compare(const TimeInstance &i1, const TimeInstance &i2)
        {
            return (i1.time < i2.time);
        }
    };

    struct SFPair
    {
        TimeInstance startInstance;
        TimeInstance startFinishInstance;
    };

    class SFOrder
    {
        TaskSetInfoDerived tasksInfo_;
        std::vector<TimeInstance> instanceOrder_;
        std::unordered_map<TimeInstance, LLint> instanceIndexMap_;
        std::unordered_map<JobCEC, SFPair> jobSFMap_;

    public:
        SFOrder() {}
        // O(n log(n))
        SFOrder(TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo)
        {
            instanceOrder_.reserve(tasksInfo.length * 2);
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                {
                    JobCEC job(i, j);
                    instanceOrder_.push_back(TimeInstance('s', GetStartTime(jobc, startTimeVector, tasksInfo), job));
                    instanceOrder_.push_back(TimeInstance('f', GetFinishTime(jobc, startTimeVector, tasksInfo), job));
                }
            }
            std::sort(instanceOrder_.begin(), instanceOrder_.end(), TimeInstance::compare);

            for (uint i = 0; i < instanceOrder_.size(); i++)
            {
            }
            // std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector = ObtainAllJobSchedule(tasksInfo, startTimeVector);
            // timeJobVector = SortJobSchedule(timeJobVector);
            // jobOrder_.reserve(timeJobVector.size());
            // for (LLint i = 0; i < static_cast<LLint>(timeJobVector.size()); i++)
            // {
            //     jobOrder_.push_back(timeJobVector[i].second);
            // }
            // UpdateIndexMap();
        }

        JobCEC RemoveJob(JobCEC job);

        void InsertStart(JobCEC job, LLint position);

        void InsertFinish(JobCEC, LLint position);
    };
} // namespace OrderOptDAG_SPACE
