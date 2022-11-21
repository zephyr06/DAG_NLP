#pragma once

#include "unordered_set"
#include "unordered_map"

#include "sources/Utils/Parameters.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"

namespace OrderOptDAG_SPACE
{

    class TimeInstance
    {
    private:
        double time;

    public:
        char type; // 's' or 'f'
        JobCEC job;
        TimeInstance(char type, JobCEC j, double t) : time(t), type(type), job(j) {}
        TimeInstance(char type, JobCEC j) : time(-1), type(type), job(j) {}
        double getTime() const
        {
            return time;
        }
        char getType() const
        {
            return type;
        }
    };
    bool compareTimeInstance(const TimeInstance i1, const TimeInstance i2);

    class SFOrder
    {
    private:
        TaskSetInfoDerived tasksInfo_;
        struct SFPair
        {
            LLint startInstanceIndex;
            LLint finishInstanceIndex;
            SFPair() : startInstanceIndex(-1), finishInstanceIndex(-1) {}
        };
        std::unordered_map<JobCEC, SFPair> jobSFMap_;
        bool whetherSFMapNeedUpdate;

    public:
        std::vector<TimeInstance> instanceOrder_;
        SFOrder() {}

        // O(n log(n))
        SFOrder(const TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo)
        {

            BeginTimer(__FUNCTION__);
            instanceOrder_.reserve(tasksInfo.length * 2);
            for (int i = 0; i < tasksInfo.N; i++)
            {
                for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++)
                {
                    JobCEC job(i, j);
                    TimeInstance instS('s', job, GetStartTime(job, startTimeVector, tasksInfo));
                    instanceOrder_.push_back(instS);
                    TimeInstance instF('f', job, GetFinishTime(job, startTimeVector, tasksInfo));
                    instanceOrder_.push_back(instF);
                }
            }

            std::sort(instanceOrder_.begin(), instanceOrder_.end(), compareTimeInstance);

            whetherSFMapNeedUpdate = true;
            EstablishJobSFMap();
            EndTimer(__FUNCTION__);
        }

        SFOrder(const TaskSetInfoDerived &tasksInfo, std::vector<TimeInstance> &instanceOrder) : tasksInfo_(tasksInfo)
        {
            instanceOrder_ = instanceOrder;
            whetherSFMapNeedUpdate = true;
        }

        // O(n)
        void EstablishJobSFMap()
        {
            BeginTimer(__FUNCTION__);
            if (!whetherSFMapNeedUpdate)
                return;
            jobSFMap_.reserve(tasksInfo_.length);
            for (size_t i = 0; i < instanceOrder_.size(); i++)
            {
                TimeInstance &inst = instanceOrder_[i];
                if (jobSFMap_.find(inst.job) == jobSFMap_.end())
                {
                    SFPair sfPair;
                    if (inst.type == 's')
                        sfPair.startInstanceIndex = i;
                    else if (inst.type == 'f')
                        sfPair.finishInstanceIndex = i;
                    else
                        CoutError("Wrong type in TimeInstance!");
                    jobSFMap_[inst.job] = sfPair;
                }
                else
                {
                    if (inst.type == 's')
                        jobSFMap_[inst.job].startInstanceIndex = i;
                    else if (inst.type == 'f')
                        jobSFMap_[inst.job].finishInstanceIndex = i;
                    else
                        CoutError("Wrong type in TimeInstance!");
                }
            }
            whetherSFMapNeedUpdate = false;
            EndTimer(__FUNCTION__);
        }

        LLint size() const { return instanceOrder_.size(); }
        TimeInstance operator[](LLint index)
        {
            RangeCheck(index);
            return instanceOrder_[index];
        }
        TimeInstance at(LLint index) const
        {
            RangeCheck(index);
            TimeInstance inst = instanceOrder_[index];
            return inst;
        }
        LLint GetJobStartInstancePosition(JobCEC &job)
        {
            EstablishJobSFMap();
            return jobSFMap_.at(job).startInstanceIndex;
        }
        LLint GetJobFinishInstancePosition(JobCEC &job)
        {
            EstablishJobSFMap();
            return jobSFMap_.at(job).finishInstanceIndex;
        }

        void RangeCheck(LLint index, bool allowEnd = false) const
        {
            if (allowEnd && (index < 0 || index > size()))
            {
                CoutError("Index error in SFOrder");
            }
            if (!allowEnd && (index < 0 || index >= size()))
                CoutError("Index error in SFOrder");
        }

        void RemoveJob(JobCEC job)
        {
            BeginTimer(__FUNCTION__);
            LLint startIndex = GetJobStartInstancePosition(job);
            LLint finishIndex = GetJobFinishInstancePosition(job);
            RangeCheck(startIndex);
            RangeCheck(finishIndex);
            instanceOrder_.erase(instanceOrder_.begin() + finishIndex);
            instanceOrder_.erase(instanceOrder_.begin() + startIndex);
            jobSFMap_.erase(job);
            // EstablishJobSFMap();
            whetherSFMapNeedUpdate = true;
            EndTimer(__FUNCTION__);
        }

        void InsertStart(JobCEC job, LLint position)
        {
            BeginTimer(__FUNCTION__);
            RangeCheck(position, true);
            TimeInstance inst('s', job);
            instanceOrder_.insert(instanceOrder_.begin() + position, inst);

            // if (jobSFMap_.find(job) == jobSFMap_.end())
            // {
            //     SFPair sfP;
            //     sfP.startInstanceIndex = position;
            //     jobSFMap_[job] = sfP;
            // }
            // else
            // {
            //     jobSFMap_[job].startInstanceIndex = position;
            // }
            // EstablishJobSFMap();
            whetherSFMapNeedUpdate = true;

            EndTimer(__FUNCTION__);
        }

        void InsertFinish(JobCEC job, LLint position)
        {
            BeginTimer(__FUNCTION__);
            RangeCheck(position, true);
            instanceOrder_.insert(instanceOrder_.begin() + position, TimeInstance('f', job));
            // EstablishJobSFMap();
            whetherSFMapNeedUpdate = true;
            EndTimer(__FUNCTION__);
        }
        void RemoveFinish(JobCEC job, LLint position)
        {
            BeginTimer(__FUNCTION__);
            RangeCheck(position, true);
            instanceOrder_.erase(instanceOrder_.begin() + position);
            // EstablishJobSFMap();
            whetherSFMapNeedUpdate = true;
            EndTimer(__FUNCTION__);
        }

        void print()
        {
            std::cout << "instanceOrder_:" << std::endl;
            for (uint i = 0; i < instanceOrder_.size(); i++)
            {
                std::cout << instanceOrder_[i].job.taskId << ", " << instanceOrder_[i].job.jobId << ", " << instanceOrder_[i].type << std::endl;
            }
        }
    };

    struct JobGroupRange // include the minIndex, but not the maxIndex
    {
        JobGroupRange(int mi, int ma) : minIndex(mi), maxIndex(ma) {}
        int minIndex;
        int maxIndex;
    };
    std::vector<TimeInstance> ExtractSubInstances(const SFOrder &jobOrderCurrForFinish, JobGroupRange &jobGroup);
} // namespace OrderOptDAG_SPACE
