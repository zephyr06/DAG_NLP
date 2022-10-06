#pragma once
#include "unordered_set"
#include "unordered_map"

#include "gtsam/base/Value.h"
#include "gtsam/inference/Symbol.h"

#include "sources/Utils/Parameters.h"
#include "sources/Tools/MatirxConvenient.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
// #include "sources/Optimization/InitialEstimate.h"

namespace DAG_SPACE
{

    class JobOrder
    {
    public:
        TaskSetInfoDerived tasksInfo_;
        std::vector<JobCEC> jobOrder_;
        std::unordered_map<JobCEC, LLint> jobOrderMap_;

        JobOrder() {}
        JobOrder(TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo)
        {
            std::vector<std::pair<std::pair<double, double>, JobCEC>> timeJobVector = ObtainAllJobSchedule(tasksInfo, startTimeVector);
            timeJobVector = SortJobSchedule(timeJobVector);
            jobOrder_.reserve(timeJobVector.size());
            for (LLint i = 0; i < static_cast<LLint>(timeJobVector.size()); i++)
            {
                jobOrder_.push_back(timeJobVector[i].second);
            }
            UpdateMap();
        }

        void UpdateMap()
        {
            for (LLint i = 0; i < static_cast<LLint>(jobOrder_.size()); i++)
            {
                jobOrderMap_[jobOrder_[i]] = i;
            }
        }

        void print() const
        {
            for (uint i = 0; i < jobOrder_.size(); i++)
            {
                std::cout << jobOrder_[i].ToString() + ", ";
                if (i % 4 == 0)
                    std::cout << std::endl;
            }
        }

        size_t size() const { return jobOrder_.size(); }

        JobCEC operator[](LLint index) { return jobOrder_[index]; }

        // jobIndex and newPosition is relative to the original job vector
        void ChangeJobOrder(LLint jobIndex, LLint newPosition)
        {
            if (jobIndex == newPosition)
                return;
            else if (jobIndex < 0 || jobIndex >= static_cast<LLint>(jobOrder_.size()) || newPosition < 0 || newPosition >= static_cast<LLint>(jobOrder_.size()))
            {
                CoutError("Index out-of-range in ChangeJobOrder");
            }

            JobCEC job = jobOrder_[jobIndex];
            jobOrder_.erase(jobOrder_.begin() + jobIndex);
            jobOrder_.insert(jobOrder_.begin() + newPosition, job);
            UpdateMap();
        }
    };

    class JobOrderMultiCore : public JobOrder
    {
    public:
        std::vector<JobCEC> jobOrderNonParall_;
        std::unordered_map<JobCEC, std::vector<JobCEC>> strictPrecedenceMap_;

        // TODO: utilize jobOrderNonParall_ and jobOrder_;
        void UpdateMap();

        void Print();

        void ChangeJobOrderNonParallel();
    };
} // namespace DAG_SPACE