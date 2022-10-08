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
        std::unordered_map<JobCEC, LLint> jobIndexMap_;

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
            UpdateIndexMap();
        }

        void UpdateIndexMap()
        {
            for (LLint i = 0; i < static_cast<LLint>(jobOrder_.size()); i++)
            {
                jobIndexMap_[jobOrder_[i]] = i;
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
            UpdateIndexMap();
        }
    };

    class JobOrderMultiCore : public JobOrder
    {
    public:
        std::vector<JobCEC> jobOrderNonParall_;
        std::unordered_map<JobCEC, LLint> jobIndexMapNP_;

        void insertNP(JobCEC jobCurr, LLint position)
        {
            if (position < 0 || position > static_cast<LLint>(jobOrderNonParall_.size()))
                CoutError("Index out-of-range in ChangeJobOrderNonParallel");
            jobOrderNonParall_.insert(jobOrderNonParall_.begin() + position, jobCurr);
            UpdateMapNP();
        }

        void eraseNP(LLint position)
        {
            if (position < 0 || position >= static_cast<LLint>(jobOrderNonParall_.size()))
                CoutError("Index out-of-range in ChangeJobOrderNonParallel");
            jobOrderNonParall_.erase(jobOrderNonParall_.begin() + position);
            UpdateMapNP();
        }
        void eraseNP(JobCEC j1)
        {
            eraseNP(jobIndexMapNP_[j1]);
        }

        void UpdateMapNP()
        {
            for (uint i = 0; i < sizeNP(); i++)
            {
                JobCEC job = jobOrderNonParall_[i];
                jobIndexMapNP_[job] = i;
            }
        }

    public:
        JobOrderMultiCore(TaskSetInfoDerived &tasksInfo, VectorDynamic &startTimeVector) : JobOrder(tasksInfo, startTimeVector) { jobOrderNonParall_.reserve(jobOrder_.size()); }

        // newPosition<0 means remove jobIndex
        /**
         * @brief Behavior of this function:
         * If jobOrderNonParall_ is empty, it inserts the job at jobIndex into jobOrderNonParall_ no matter what newPosition's value is;
         * Otherwise, it switches position similarly as ChangeJobOrder;
         *
         * @param jobIndex
         * @param newPosition
         */
        void ChangeJobOrderNonParallel(LLint jobIndex, LLint newPosition)
        {
            if (jobIndex < 0 || jobIndex >= static_cast<LLint>(jobOrder_.size()) || newPosition > static_cast<LLint>(jobOrderNonParall_.size()))
            {
                CoutError("Index out-of-range in ChangeJobOrderNonParallel");
            }

            JobCEC jobCurr = jobOrder_[jobIndex];

            if (jobIndex == newPosition)
                return;
            if (newPosition < 0)
            {
                eraseNP(jobIndex);
                return;
            }
            JobCEC job = jobOrderNonParall_[jobIndex];
            eraseNP(jobIndex);
            insertNP(job, newPosition);
        }

        size_t sizeNP() const { return jobOrderNonParall_.size(); }
    };
} // namespace DAG_SPACE