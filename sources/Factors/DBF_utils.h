#pragma once
#include "sources/Factors/Interval.h"
#include "sources/Optimization/EliminationForest_utils.h"

namespace OrderOptDAG_SPACE
{
    using namespace RegularTaskSystem;
    /**
     * @brief get c_{ijk} according to ILP paper
     *
     * @param startTime_i
     * @param task_i
     * @param startTime_j
     * @param task_j
     * @param startTime_k
     * @param task_k
     * @return double
     */
    inline double ComputationTime_IJK(double startTime_i, const Task &task_i, double startTime_j,
                                      const Task &task_j, double startTime_k, const Task &task_k)
    {
        if (startTime_i <= startTime_k && startTime_k + task_k.executionTime <= startTime_j + task_j.executionTime)
        {
            return task_k.executionTime;
        }
        else
            return 0;
    }
    inline double ComputationTime_IJK(double startTime_i, double startTime_j,
                                      double length_j, double startTime_k, double length_k)
    {
        if (startTime_i <= startTime_k && startTime_k + length_k <= startTime_j + length_j)
        {
            return length_k;
        }
        else
            return 0;
    }
    inline double ComputationTime_IJK(Interval &i_i, Interval &i_j, Interval &i_k)
    {
        if (i_i.start <= i_k.start && i_k.start + i_k.length <= i_j.start + i_j.length)
        {
            return i_k.length;
        }
        else
            return 0;
    }
    inline Interval CreateSingleInterval(LLint index, double start,
                                         const TaskSet &tasks,
                                         const vector<LLint> &sizeOfVariables)
    {
        int taskId = BigIndex2TaskIndex(index, sizeOfVariables);
        double length = tasks[taskId].executionTime;
        int coreRequire = tasks[taskId].coreRequire;
        return Interval{start, length, index, coreRequire};
    }
    /**
     * @brief Create a Interval vector for indexes specified by the tree1 parameter;
     * the return interval follows the same order given by tree1!
     *
     * @param tree1 index in startTimeVector
     * @param startTimeVector
     * @return vector<Interval>
     */
    vector<Interval> CreateIntervalFromSTVSameOrder(vector<LLint> &tree1, const VectorDynamic &startTimeVector,
                                                    const TaskSet &tasks,
                                                    const vector<LLint> &sizeOfVariables)
    {
        LLint n = tree1.size();
        vector<Interval> intervalVec;
        intervalVec.reserve(n);
        for (LLint i = 0; i < n; i++)
        {
            LLint index = tree1[i];
            double start = startTimeVector.coeff(index, 0);
            Interval inv = CreateSingleInterval(index, start, tasks, sizeOfVariables);
            intervalVec.push_back(inv);
        }
        return intervalVec;
    }

    vector<Interval> DbfInterval(const VectorDynamic &startTimeVector, int processorId,
                                 const ProcessorTaskSet &processorTasks,
                                 const TaskSet &tasks,
                                 const vector<LLint> &sizeOfVariables)
    {
        vector<LLint> indexes;
        indexes.reserve(startTimeVector.size());
        for (int taskId : processorTasks.at(processorId))
        {
            for (int j = 0; j < sizeOfVariables[taskId]; j++)
            {
                indexes.push_back(IndexTran_Instance2Overall(taskId, j, sizeOfVariables));
            }
        }
        vector<Interval> intervalVec = CreateIntervalFromSTVSameOrder(indexes,
                                                                      startTimeVector, tasks, sizeOfVariables);

        return intervalVec;
    }

    double DbfIntervalOverlapError(const VectorDynamic &startTimeVector, int processorId,
                                   const ProcessorTaskSet &processorTasks,
                                   const TaskSet &tasks,
                                   const vector<LLint> &sizeOfVariables)
    {
        vector<Interval> intervalVec = DbfInterval(startTimeVector, processorId,
                                                   processorTasks, tasks, sizeOfVariables);
        return IntervalOverlapError(intervalVec);
    }

    /**
     * @brief FindVanishIndex; given a startTimeVector, some of their intervals may be fully overlapped
     * by another, and this function finds all the indexes that is related.
     *
     * @param startTimeVectorOrig
     * @return vector<LLint>
     */
    vector<LLint> FindVanishIndex(const VectorDynamic &startTimeVectorOrig,
                                  const TaskSet &tasks,
                                  const vector<LLint> &sizeOfVariables,
                                  const EliminationForest &forestInfo)
    {
        BeginTimer("FindVanishIndex");
        VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                               forestInfo);
        vector<LLint> indexes;
        indexes.reserve(startTimeVector.size());
        for (uint i = 0; i < startTimeVector.size(); i++)
            indexes.push_back(i);
        vector<Interval> intervalVec = CreateIntervalFromSTVSameOrder(indexes, startTimeVector, tasks, sizeOfVariables);

        LLint variableDimension = intervalVec.size();
        vector<LLint> coverIntervalIndex;
        coverIntervalIndex.reserve(variableDimension);
        std::unordered_set<LLint> indexSetBig;

        for (LLint i = 0; i < variableDimension; i++)
        {
            double s1 = intervalVec[i].start;
            double f1 = s1 + intervalVec[i].length;
            for (LLint j = i + 1; j < variableDimension; j++)
            {
                double s2 = intervalVec[j].start;
                double f2 = s2 + intervalVec[j].length;

                if ((s2 > s1 && f2 < f1) || (s2 < s1 && f2 > f1))
                {
                    LLint leafIndex = FindLeaf(i, forestInfo.mapIndex);
                    if (indexSetBig.find(leafIndex) == indexSetBig.end())
                    {
                        indexSetBig.insert(leafIndex);
                    }
                    leafIndex = FindLeaf(j, forestInfo.mapIndex);
                    if (indexSetBig.find(leafIndex) == indexSetBig.end())
                    {
                        indexSetBig.insert(leafIndex);
                    }
                }
            }
        }

        auto m = MapIndex_True2Compress(forestInfo.maskForEliminate);
        vector<LLint> coverIndexInCompressed;
        coverIndexInCompressed.reserve(startTimeVectorOrig.rows());
        for (auto itr = indexSetBig.begin(); itr != indexSetBig.end(); itr++)
        {
            coverIndexInCompressed.push_back(m[(*itr)]);
        }
        EndTimer("FindVanishIndex");
        return coverIndexInCompressed;
    }

}