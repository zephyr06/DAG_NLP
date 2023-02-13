#include "sources/Optimization/SFOrder.h"

namespace OrderOptDAG_SPACE {

bool compareTimeInstance(const TimeInstance i1, const TimeInstance i2) {
  if (std::abs(i1.getTime() - i2.getTime()) > GlobalVariablesDAGOpt::LPTolerance / 10)
    return (i1.getTime() < i2.getTime());
  else {
    if (i1.type != i2.type) {
      return i1.type == 'f';
    } else {
      return i1.job.taskId < i2.job.taskId;
    }
  }
}

std::vector<TimeInstance> ExtractSubInstances(const SFOrder &jobOrderCurrForFinish, JobGroupRange &jobGroup) {
  // BeginTimer("ExtractSubInstances");
  // BeginTimer(__FUNCTION__);
  struct JobInstanceInfo {
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
  for (size_t i = minIndex; i < maxIndex; i++) {

    TimeInstance instCurr = jobOrderCurrForFinish.at(i);
    JobCEC jobCurr = instCurr.job;
    JobInstanceInfo info;
    auto itr = jobMap.find(jobCurr);
    if (itr != jobMap.end()) {
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
  for (size_t i = minIndex; i < maxIndex; i++) {
    TimeInstance instCurr = jobOrderCurrForFinish.at(i);
    JobCEC &jobCurr = instCurr.job;
    if (jobMap[jobCurr].valid())
      instanceOrderSmall.push_back(instCurr);
    // else
    //     int a = 1;
  }
  // EndTimer(__FUNCTION__);
  // EndTimer("ExtractSubInstances");
  return instanceOrderSmall;
}

SFOrder::SFOrder(const SFOrder &jobOrder) // for convenience of profiling
{
#ifdef PROFILE_CODE
  BeginTimer(__FUNCTION__);
#endif
  tasksInfo_ = jobOrder.tasksInfo_;
  jobSFMap_ = jobOrder.jobSFMap_;
  instanceOrder_ = jobOrder.instanceOrder_;
  whetherSFMapNeedUpdate = jobOrder.whetherSFMapNeedUpdate;
#ifdef PROFILE_CODE
  EndTimer(__FUNCTION__);
#endif
}
SFOrder &SFOrder::operator=(const SFOrder &jobOrder) {
#ifdef PROFILE_CODE
  BeginTimer("SFOrder");
#endif
  tasksInfo_ = jobOrder.tasksInfo_;
  jobSFMap_ = jobOrder.jobSFMap_;
  instanceOrder_ = jobOrder.instanceOrder_;
  whetherSFMapNeedUpdate = jobOrder.whetherSFMapNeedUpdate;
#ifdef PROFILE_CODE
  EndTimer("SFOrder");
#endif
  return *this;
}

void SFOrder::EstablishJobSFMap() {
#ifdef PROFILE_CODE
  BeginTimer(__FUNCTION__);
#endif
  if (!whetherSFMapNeedUpdate) {
#ifdef PROFILE_CODE
    EndTimer(__FUNCTION__);
#endif
    return;
  }
  jobSFMap_.reserve(tasksInfo_.length);
  for (size_t i = 0; i < instanceOrder_.size(); i++) {
    TimeInstance &inst = instanceOrder_[i];
    if (jobSFMap_.find(inst.job) == jobSFMap_.end()) {
      SFPair sfPair;
      if (inst.type == 's')
        sfPair.startInstanceIndex = i;
      else if (inst.type == 'f')
        sfPair.finishInstanceIndex = i;
      else
        CoutError("Wrong type in TimeInstance!");
      jobSFMap_[inst.job] = sfPair;
    } else {
      if (inst.type == 's')
        jobSFMap_[inst.job].startInstanceIndex = i;
      else if (inst.type == 'f')
        jobSFMap_[inst.job].finishInstanceIndex = i;
      else
        CoutError("Wrong type in TimeInstance!");
    }
  }
  whetherSFMapNeedUpdate = false;
#ifdef PROFILE_CODE
  EndTimer(__FUNCTION__);
#endif
}

void SFOrder::RangeCheck(LLint index, bool allowEnd) const {
  if (allowEnd && (index < 0 || index > size())) {
    CoutError("Index error in SFOrder");
  }
  if (!allowEnd && (index < 0 || index >= size()))
    CoutError("Index error in SFOrder");
}

void SFOrder::RemoveJob(JobCEC job) {
  LLint startIndex = GetJobStartInstancePosition(job);
  LLint finishIndex = GetJobFinishInstancePosition(job);
  RangeCheck(startIndex);
  RangeCheck(finishIndex);
  instanceOrder_.erase(instanceOrder_.begin() + finishIndex);
  instanceOrder_.erase(instanceOrder_.begin() + startIndex);
  jobSFMap_.erase(job);
  // EstablishJobSFMap();
  whetherSFMapNeedUpdate = true;
}

void SFOrder::InsertStart(JobCEC job, LLint position) {
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
}

void SFOrder::InsertFinish(JobCEC job, LLint position) {
  RangeCheck(position, true);
  instanceOrder_.insert(instanceOrder_.begin() + position, TimeInstance('f', job));
  // EstablishJobSFMap();
  whetherSFMapNeedUpdate = true;
}
void SFOrder::RemoveStart(JobCEC job, LLint position) {
  RangeCheck(position, true);
  if (job == instanceOrder_.at(position).job)
    RemoveInstance(job, position);
  else {
    LLint positionActual = GetJobStartInstancePosition(job);
    RemoveInstance(job, positionActual);
  }
}

void SFOrder::RemoveFinish(JobCEC job, LLint position) {
  RangeCheck(position, true);
  if (job == instanceOrder_.at(position).job)
    RemoveInstance(job, position);
  else {
    LLint positionActual = GetJobFinishInstancePosition(job);
    RemoveInstance(job, positionActual);
  }
}
void SFOrder::RemoveInstance(JobCEC job, LLint position) {
  RangeCheck(position, true);
  if (job != instanceOrder_.at(position).job)
    CoutError("Wrong position while removing instance!");
  instanceOrder_.erase(instanceOrder_.begin() + position);
  // EstablishJobSFMap();
  whetherSFMapNeedUpdate = true;
}

void SFOrder::print() const {
  std::cout << "instanceOrder_:" << std::endl;
  for (uint i = 0; i < instanceOrder_.size(); i++) {
    std::cout << instanceOrder_[i].job.taskId << ", " << instanceOrder_[i].job.jobId << ", "
              << instanceOrder_[i].type << std::endl;
  }
}

bool SFOrder::operator==(const SFOrder &anotherJobOrder) const {
  if (size() != anotherJobOrder.size())
    return false;
  for (uint i = 0; i < instanceOrder_.size(); i++) {
    if (instanceOrder_[i].job != anotherJobOrder.at(i).job ||
        instanceOrder_[i].type != anotherJobOrder.at(i).type)
      return false;
  }
  return true;
}
bool SFOrder::operator!=(const SFOrder &anotherJobOrder) const { return !(*this == anotherJobOrder); }

} // namespace OrderOptDAG_SPACE
