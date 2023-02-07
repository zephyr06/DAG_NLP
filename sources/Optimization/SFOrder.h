#pragma once

#include "unordered_map"
#include "unordered_set"

#include "sources/TaskModel/DAG_Model.h"
#include "sources/Utils/JobCEC.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"

namespace OrderOptDAG_SPACE {

class TimeInstance {
private:
  double time;

public:
  char type; // 's' or 'f'
  JobCEC job;
  TimeInstance(char type, JobCEC j, double t) : time(t), type(type), job(j) {}
  TimeInstance(char type, JobCEC j) : time(-1), type(type), job(j) {}
  double getTime() const { return time; }
  char getType() const { return type; }

  inline bool operator==(const TimeInstance &b) const { return type == b.type && job == b.job; }
  inline bool operator!=(const TimeInstance &b) const { return !(*this == b); }

  double GetRangeMin(const TaskSetInfoDerived &tasksInfo) const {
    if (type == 's')
      return GetActivationTime(job, tasksInfo);
    else if (type == 'f')
      return GetActivationTime(job, tasksInfo) + GetExecutionTime(job, tasksInfo);
    else
      CoutError("Wrong type!");
    return -1;
  }

  double GetRangeMax(const TaskSetInfoDerived &tasksInfo) const {
    if (type == 's')
      return GetDeadline(job, tasksInfo) - GetExecutionTime(job, tasksInfo);
    else if (type == 'f')
      return GetDeadline(job, tasksInfo);
    else
      CoutError("Wrong type!");
    return -1;
  }
};

bool compareTimeInstance(const TimeInstance i1, const TimeInstance i2);

class SFOrder {
private:
  TaskSetInfoDerived tasksInfo_;
  struct SFPair {
    LLint startInstanceIndex;
    LLint finishInstanceIndex;
    SFPair() : startInstanceIndex(-1), finishInstanceIndex(-1) {}
  };
  std::unordered_map<JobCEC, SFPair> jobSFMap_;

public:
  std::vector<TimeInstance> instanceOrder_;
  bool whetherSFMapNeedUpdate;
  SFOrder() {}

  // O(n log(n))
  SFOrder(const TaskSetInfoDerived &tasksInfo, const VectorDynamic &startTimeVector) : tasksInfo_(tasksInfo) {

    instanceOrder_.reserve(tasksInfo.length * 2);
    for (int i = 0; i < tasksInfo.N; i++) {
      for (uint j = 0; j < tasksInfo.sizeOfVariables[i]; j++) {
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
  }

  SFOrder(const TaskSetInfoDerived &tasksInfo, std::vector<TimeInstance> &instanceOrder)
      : tasksInfo_(tasksInfo) {
    instanceOrder_ = instanceOrder;
    whetherSFMapNeedUpdate = true;
  }

  // O(n)
  void EstablishJobSFMap();

  LLint size() const { return instanceOrder_.size(); }
  TimeInstance operator[](LLint index) {
    RangeCheck(index);
    return instanceOrder_[index];
  }

  TimeInstance at(LLint index) const {
    RangeCheck(index);
    TimeInstance inst = instanceOrder_[index];
    return inst;
  }

  LLint GetJobStartInstancePosition(const JobCEC &job) {
    EstablishJobSFMap();
    if (job.jobId >= tasksInfo_.sizeOfVariables[job.taskId]) {
      JobCEC jobWithinSingleHP{job.taskId, job.jobId % tasksInfo_.sizeOfVariables[job.taskId]};
      return jobSFMap_.at(jobWithinSingleHP).startInstanceIndex +
             job.jobId / tasksInfo_.sizeOfVariables[job.taskId] * tasksInfo_.length * 2;
    } else
      return jobSFMap_.at(job).startInstanceIndex;
  }
  LLint GetJobFinishInstancePosition(const JobCEC &job) {
    EstablishJobSFMap();
    if (job.jobId >= tasksInfo_.sizeOfVariables[job.taskId]) {
      JobCEC jobWithinSingleHP{job.taskId, job.jobId % tasksInfo_.sizeOfVariables[job.taskId]};
      return jobSFMap_.at(jobWithinSingleHP).finishInstanceIndex +
             job.jobId / tasksInfo_.sizeOfVariables[job.taskId] * tasksInfo_.length * 2;
    } else
      return jobSFMap_.at(job).finishInstanceIndex;
  }
  JobPosition GetJobPosition(const JobCEC &job) {
    return JobPosition(GetJobStartInstancePosition(job), GetJobFinishInstancePosition(job));
  }

  void RangeCheck(LLint index, bool allowEnd = false) const;

  void RemoveJob(JobCEC job);

  void InsertStart(JobCEC job, LLint position);

  void InsertFinish(JobCEC job, LLint position);

  void RemoveInstance(JobCEC job, LLint position);

  void RemoveStart(JobCEC job, LLint position);
  void RemoveFinish(JobCEC job, LLint position);

  void print() const;

  bool operator==(const SFOrder &anotherJobOrder) const;
  bool operator!=(const SFOrder &anotherJobOrder) const;
};

struct JobGroupRange // include the minIndex, but not the maxIndex
{
  JobGroupRange(int mi, int ma) : minIndex(mi), maxIndex(ma) {}
  int minIndex;
  int maxIndex;
};
std::vector<TimeInstance> ExtractSubInstances(const SFOrder &jobOrderCurrForFinish, JobGroupRange &jobGroup);
} // namespace OrderOptDAG_SPACE
