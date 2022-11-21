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
} // namespace OrderOptDAG_SPACE
