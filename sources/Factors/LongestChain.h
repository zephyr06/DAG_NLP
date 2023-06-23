#pragma once
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Optimization/ScheduleOptions.h"

namespace OrderOptDAG_SPACE {

// sourceJob is not used because it is equivalent as jobChain[0]
double GetReactionTime(const std::vector<JobCEC> &jobChain,
                       const VectorDynamic &startTimeVector,
                       const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

std::vector<std::vector<JobCEC>> GetMaxReactionTimeChains(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &react_chain_map,
    const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    double tolerance = 1e-3);

double GetDataAge(const std::vector<JobCEC> &jobChain,
                  const VectorDynamic &startTimeVector,
                  const RegularTaskSystem::TaskSetInfoDerived &tasksInfo);

std::vector<std::vector<JobCEC>> GetMaxDataAgeChains(
    const std::unordered_map<JobCEC, std::vector<JobCEC>> &da_chain_map,
    const VectorDynamic &startTimeVector,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    double tolerance = 1e-3);

class LongestCAChain {
   public:
    LongestCAChain() {}
    LongestCAChain(const DAG_Model &dagTasks,
                   const TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder,
                   const VectorDynamic &startTimeVector, int processorNum,
                   std::string type)
        : obj_type_trait_(type),
          longestChains_(FindLongestCAChain(dagTasks, tasksInfo, jobOrder,
                                            startTimeVector, processorNum)) {}

    std::vector<std::vector<JobCEC>> FindLongestCAChain(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum);

    size_t size() const { return longestChains_.size(); }
    // inline begin() const { return longestChains_[0]; }
    // inline back() const { return longestChains_[size() - 1]; }

    std::vector<JobCEC> operator[](size_t index) const {
        return longestChains_[index];
    }

    std::vector<std::vector<JobCEC>> GetLongestJobChains_RT(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum);

    std::vector<std::vector<JobCEC>> GetLongestJobChains_DA(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum);

    std::vector<std::vector<JobCEC>> GetLongestJobChains_SF(
        const DAG_Model &dagTasks, const TaskSetInfoDerived &tasksInfo,
        SFOrder &jobOrder, const VectorDynamic &startTimeVector,
        int processorNum);

    // data members
    std::string obj_type_trait_;
    std::vector<std::vector<JobCEC>> longestChains_;
};

int FindSiblingJobIndex(const JobCEC &job,
                        const std::vector<JobCEC> &jobChainCurr);
// The input jobOrder is the same as jobOrderRef
// it assumes the input jobOrder will first remove job, and then insert its
// start/finish instances at startP/finishP If you want to be safer, return more
// 'true'
bool WhetherJobBreakChainRT(const JobCEC &job, LLint startP, LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo);
bool WhetherJobBreakChainDA(const JobCEC &job, LLint startP, LLint finishP,
                            const LongestCAChain &longestJobChains,
                            const DAG_Model &dagTasks, SFOrder &jobOrder,
                            const TaskSetInfoDerived &tasksInfo);

bool WhetherJobBreakChain(const JobCEC &job, LLint startP, LLint finishP,
                          const LongestCAChain &longestJobChains,
                          const DAG_Model &dagTasks, SFOrder &jobOrder,
                          const TaskSetInfoDerived &tasksInfo,
                          std::string obj_type);

std::unordered_map<JobCEC, int> ExtractIndependentJobGroups(
    const SFOrder &jobOrder, const TaskSetInfoDerived &tasksInfo);

inline bool WhetherInfluenceJobSimple(
    const JobCEC &jobCurr, const JobCEC &jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap) {
    return (jobGroupMap[jobCurr] == jobGroupMap[jobChanged]);
}

bool WhetherInfluenceJobSource(
    JobCEC jobCurr, const JobCEC &jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    LLint startP, LLint finishP,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);
bool WhetherInfluenceJobSink(
    JobCEC jobCurr, const JobCEC &jobChanged,
    std::unordered_map<JobCEC, int> &jobGroupMap, SFOrder &jobOrder,
    LLint startP, LLint finishP,
    const RegularTaskSystem::TaskSetInfoDerived &tasksInfo,
    const VectorDynamic &startTimeVector);
}  // namespace OrderOptDAG_SPACE