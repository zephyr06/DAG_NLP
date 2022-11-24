#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE
{

    bool compareTimeInstance(const TimeInstance i1, const TimeInstance i2)
    {
        if (i1.getTime() != i2.getTime())
            return (i1.getTime() < i2.getTime());
        else
        {
            if (i1.type != i2.type)
            {
                return i1.type == 'f';
            }
            else
            {
                return i1.job.taskId < i2.job.taskId;
            }
        }
    }

    std::vector<TimeInstance> ExtractSubInstances(const SFOrder &jobOrderCurrForFinish, JobGroupRange &jobGroup)
    {
        BeginTimer("ExtractSubInstances");
        // BeginTimer(__FUNCTION__);
        struct JobInstanceInfo
        {
            int start;
            int finish;
            JobInstanceInfo() : start(-1), finish(-1) {}
            bool valid() const { return start != -1 && finish != -1; }
        };
        std::unordered_map<JobCEC, JobInstanceInfo> jobMap;

        size_t minIndex = jobGroup.minIndex;
        size_t maxIndex = jobGroup.maxIndex;
        //  first loop, establish jobMap
        std::unordered_set<JobCEC> instSet;
        for (size_t i = minIndex; i < maxIndex; i++)
        {

            TimeInstance instCurr = jobOrderCurrForFinish.at(i);
            JobCEC jobCurr = instCurr.job;
            JobInstanceInfo info;
            auto itr = jobMap.find(jobCurr);
            if (itr != jobMap.end())
            {
                info = itr->second;
            }

            if (instCurr.type == 's')
                info.start = i;
            else if (instCurr.type == 'f')
                info.finish = i;
            else
                CoutError("ExtractSubInstances: unrecognized type in TimeInstance!");

            jobMap[jobCurr] = info;
        }

        // second loop, obtain instanceOrderSmall based on jobMap
        std::vector<TimeInstance> instanceOrderSmall;
        instanceOrderSmall.reserve(jobGroup.maxIndex - jobGroup.minIndex);
        for (size_t i = minIndex; i < maxIndex; i++)
        {
            TimeInstance instCurr = jobOrderCurrForFinish.at(i);
            JobCEC &jobCurr = instCurr.job;
            if (jobMap[jobCurr].valid())
                instanceOrderSmall.push_back(instCurr);
            // else
            //     int a = 1;
        }
        // EndTimer(__FUNCTION__);
        EndTimer("ExtractSubInstances");
        return instanceOrderSmall;
    }

    void SFOrder::EstablishJobSFMap()
    {
        BeginTimer(__FUNCTION__);
        if (!whetherSFMapNeedUpdate)
        {
            return;
            EndTimer(__FUNCTION__);
        }
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

    void SFOrder::RangeCheck(LLint index, bool allowEnd) const
    {
        if (allowEnd && (index < 0 || index > size()))
        {
            CoutError("Index error in SFOrder");
        }
        if (!allowEnd && (index < 0 || index >= size()))
            CoutError("Index error in SFOrder");
    }

    void SFOrder::RemoveJob(JobCEC job)
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

    void SFOrder::InsertStart(JobCEC job, LLint position)
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

    void SFOrder::InsertFinish(JobCEC job, LLint position)
    {
        BeginTimer(__FUNCTION__);
        RangeCheck(position, true);
        instanceOrder_.insert(instanceOrder_.begin() + position, TimeInstance('f', job));
        // EstablishJobSFMap();
        whetherSFMapNeedUpdate = true;
        EndTimer(__FUNCTION__);
    }
    void SFOrder::RemoveFinish(JobCEC job, LLint position)
    {
        BeginTimer(__FUNCTION__);
        RangeCheck(position, true);
        instanceOrder_.erase(instanceOrder_.begin() + position);
        // EstablishJobSFMap();
        whetherSFMapNeedUpdate = true;
        EndTimer(__FUNCTION__);
    }

    void SFOrder::print()
    {
        std::cout << "instanceOrder_:" << std::endl;
        for (uint i = 0; i < instanceOrder_.size(); i++)
        {
            std::cout << instanceOrder_[i].job.taskId << ", " << instanceOrder_[i].job.jobId << ", " << instanceOrder_[i].type << std::endl;
        }
    }

} // namespace OrderOptDAG_SPACE
