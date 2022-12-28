
#include "sources/Factors/SensorFusionFactor.h"

namespace OrderOptDAG_SPACE
{
    LLint CountSFError(const DAG_Model &dagTasks, const std::vector<LLint> &sizeOfVariables)
    {
        LLint errorDimensionSF = 0;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            if ((itr->second).size() > 1)
                errorDimensionSF += sizeOfVariables[(itr->first)];
        }
        return errorDimensionSF;
    }

    std::pair<int, int> ExtractMaxDistance(std::vector<IndexData> &sourceFinishTime)
    {
        // return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
        //        *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
        if (sourceFinishTime.size() == 0)
            return std::make_pair(0, 0);

        double maxEle = sourceFinishTime[0].time;
        int maxIndex = 0;
        double minEle = sourceFinishTime[0].time;
        int minIndex = 0;

        for (uint i = 0; i < sourceFinishTime.size(); i++)
        {
            if (maxEle < sourceFinishTime[i].time)
            {
                maxIndex = i;
                maxEle = sourceFinishTime[i].time;
            }
            if (minEle > sourceFinishTime[i].time)
            {
                minIndex = i;
                minIndex = sourceFinishTime[i].time;
            }
        }
        return std::make_pair(maxIndex, minIndex);
    }

    VectorDynamic ObtainSensorFusionError(const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector)
    {
        VectorDynamic res;
        LLint errorDimension = CountSFError(dagTasks, tasksInfo.sizeOfVariables);
        if (errorDimension == 0)
            return GenerateVectorDynamic1D(0);

        res.resize(errorDimension, 1);
        LLint indexRes = 0;
        for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
        {
            const TaskSet &tasksPrev = itr->second;
            if (tasksPrev.size() > 1)
            {
                std::vector<double> sourceFinishTime;
                sourceFinishTime.reserve(tasksPrev.size());
                size_t indexCurr = itr->first;

                for (int instanceCurr = 0; instanceCurr < tasksInfo.sizeOfVariables[indexCurr]; instanceCurr++)
                {
                    sourceFinishTime.clear();
                    double startTimeCurr = ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, indexCurr, instanceCurr);

                    // go through three source sensor tasks in the example
                    for (size_t ii = 0; ii < tasksPrev.size(); ii++)
                    {
                        int sourceIndex = tasksPrev.at(ii).id;
                        LLint instanceSource = floor(startTimeCurr / tasksInfo.tasks[sourceIndex].period);
                        if (instanceSource < 0 || instanceSource > tasksInfo.sizeOfVariables[sourceIndex] - 1)
                        // CoutError("Error in OptimizeORder's SF evaluation!");
                        {
                            if (GlobalVariablesDAGOpt::debugMode == 1)
                                CoutWarning("Possible Error in OptimizeORder's SF evaluation!");
                            instanceSource = 0;
                        }

                        JobCEC jCurr(sourceIndex, instanceSource);
                        double finishTimeSourceInstance = GetFinishTime(jCurr, startTimeVector, tasksInfo);
                        if (finishTimeSourceInstance <= startTimeCurr)
                            sourceFinishTime.push_back(finishTimeSourceInstance);
                        else
                        {
                            if (jCurr.jobId == 0)
                                jCurr.jobId = tasksInfo.sizeOfVariables[jCurr.taskId] - 1;
                            else
                                jCurr.jobId--;
                            sourceFinishTime.push_back(GetFinishTime(jCurr, startTimeVector, tasksInfo));
                        }
                    }
                    res(indexRes, 0) = ExtractMaxDistance(sourceFinishTime);
                    indexRes++;
                }
            }
        }
        return res;
    }

    std::unordered_map<JobCEC, std::vector<JobCEC>> GetSensorMapFromSingleJob(
        const TaskSetInfoDerived &tasksInfo, const int task_id, const TaskSet &precede_tasks, const VectorDynamic &x)
    {
        std::unordered_map<JobCEC, std::vector<JobCEC>> sensor_map;

        if (precede_tasks.size() < 2)
        {
            return sensor_map;
        }
        LLint hyperPeriod = tasksInfo.hyperPeriod;
        const TaskSet &tasks = tasksInfo.tasks;
        LLint totalStartJobs = hyperPeriod / tasks[task_id].period + 1;

        for (LLint startInstanceIndex = 0; startInstanceIndex < totalStartJobs; startInstanceIndex++)
        {
            JobCEC succeed_job = {task_id, (startInstanceIndex)};
            std::vector<JobCEC> precede_jobs;
            double currentJobST = GetStartTime(succeed_job, x, tasksInfo);
            for (auto precede_task : precede_tasks)
            {
                LLint jobIndex = 0;
                while (GetFinishTime({precede_task.id, jobIndex}, x, tasksInfo) <= currentJobST)
                {
                    jobIndex++;
                    if (jobIndex > 10000)
                    {
                        CoutError("didn't find a match!");
                    }
                }
                if (jobIndex > 0)
                {
                    precede_jobs.push_back(JobCEC(precede_task.id, jobIndex - 1));
                }
            }
            if (precede_jobs.size() > 0)
            {
                sensor_map[succeed_job] = precede_jobs;
            }
        }
        return sensor_map;
    }
}