#pragma once
#include <algorithm> // for copy() and assign()
#include <iterator>  // for back_inserter

#include "sources/Tools/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/JobOrder.h"
#include "sources/Factors/RTDA_Factor.h"
#include "sources/Factors/DBF_ConstraintFactorNonPreemptive.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Utils/OptimizeOrderUtils.h"
#include "sources/Factors/Interval.h"
#include "sources/Optimization/ScheduleOptimizer.h"
#include "sources/Factors/SensorFusionFactor.h"
#include "sources/Optimization/SFOrder.h"
// #include "sources/Tools/profilier.h"

namespace OrderOptDAG_SPACE
{
    namespace OptimizeSF
    {
        std::vector<int> GetTaskIdWithChainOrder(DAG_Model &dagTasks)
        {
            std::vector<int> idVec;
            idVec.reserve(dagTasks.tasks.size());
            if (dagTasks.chains_.size() == 0)
            {
                return idVec;
            }
            for (uint i = 0; i < dagTasks.chains_.size(); i++)
                std::copy(dagTasks.chains_[i].begin(), dagTasks.chains_[i].end(), back_inserter(idVec));
            if (enableFastSearch)
                return idVec;

            unordered_set<int> idSet;
            std::vector<int> idVecChainFirst = dagTasks.chains_[0];
            for (uint i = 0; i < dagTasks.chains_[0].size(); i++)
                idSet.insert(dagTasks.chains_[0][i]);

            for (uint i = 0; i < dagTasks.tasks.size(); i++)
            {
                if (idSet.find(i) == idSet.end())
                    idVec.push_back(i);
            }
            return idVec;
        }
        struct IterationStatus
        {
            DAG_Model dagTasks_;
            SFOrder &jobOrder_;
            int processorNum_;
            VectorDynamic startTimeVector_;
            std::vector<uint> processorJobVec_;
            std::vector<std::vector<RTDA>> rtdaVec_; // for each chain
            std::vector<RTDA> maxRtda_;              // for each chain
            // double objVal_;
            bool schedulable_; // only basic schedulability
            VectorDynamic sfVec_;
            double objWeighted_;

            IterationStatus(DAG_Model &dagTasks, TaskSetInfoDerived &tasksInfo, SFOrder &jobOrder, int processorNum) : dagTasks_(dagTasks), jobOrder_(jobOrder), processorNum_(processorNum)
            {
                // startTimeVector_ = ListSchedulingGivenOrder(dagTasks, tasksInfo, jobOrder_);
                BeginTimerAppInProfiler;
                processorJobVec_.clear();
                startTimeVector_ = SFOrderScheduling(dagTasks, tasksInfo, processorNum_, jobOrder_, processorJobVec_);

                for (uint i = 0; i < dagTasks.chains_.size(); i++)
                {
                    auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                    rtdaVec_.push_back(rtdaVecTemp);
                    maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
                }
                if (considerSensorFusion)
                {
                    sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                }
                schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_);
                objWeighted_ = ObjWeighted();
                if (doScheduleOptimization && !doScheduleOptimizationOnlyOnce)
                {
                    ScheduleResult scheduleResBeforeOpt{jobOrder_, startTimeVector_, schedulable_, ReadObj(), processorJobVec_};
                    scheduleResBeforeOpt.objWeighted_ = objWeighted_;

                    ScheduleResult resultAfterOptimization;
                    ScheduleOptimizer scheduleOptimizer = ScheduleOptimizer();
                    scheduleOptimizer.OptimizeObjWeighted(dagTasks, scheduleResBeforeOpt);
                    resultAfterOptimization = scheduleOptimizer.getOptimizedResult();
                    if (resultAfterOptimization.objWeighted_ < scheduleResBeforeOpt.objWeighted_)
                    {
                        startTimeVector_ = resultAfterOptimization.startTimeVector_;
                        rtdaVec_.clear();
                        maxRtda_.clear();
                        for (uint i = 0; i < dagTasks.chains_.size(); i++)
                        {
                            auto rtdaVecTemp = GetRTDAFromSingleJob(tasksInfo, dagTasks.chains_[i], startTimeVector_);
                            rtdaVec_.push_back(rtdaVecTemp);
                            maxRtda_.push_back(GetMaxRTDA(rtdaVecTemp));
                        }
                        if (considerSensorFusion)
                        {
                            sfVec_ = ObtainSensorFusionError(dagTasks_, tasksInfo, startTimeVector_);
                        }
                        schedulable_ = ExamBasic_Feasibility(dagTasks, tasksInfo, startTimeVector_, processorJobVec_, processorNum_);
                        // TODO: remove this?
                        jobOrder_ = SFOrder(tasksInfo, startTimeVector_);
                        objWeighted_ = ObjWeighted();
                    }
                }
                EndTimerAppInProfiler;
            }

            IterationStatus &operator=(IterationStatus &status)
            {
                dagTasks_ = status.dagTasks_;
                jobOrder_ = status.jobOrder_;
                processorNum_ = status.processorNum_;
                startTimeVector_ = status.startTimeVector_;
                processorJobVec_ = status.processorJobVec_;
                rtdaVec_ = status.rtdaVec_;
                maxRtda_ = status.maxRtda_;
                schedulable_ = status.schedulable_;
                sfVec_ = status.sfVec_;
                objWeighted_ = status.objWeighted_;
                return *this;
            }

            double ReadObj()
            {
                double res = ObjRTDA(maxRtda_);
                return res;
            }
            double ObjWeighted()
            {
                double overallRTDA = 0;
                for (uint i = 0; i < rtdaVec_.size(); i++)
                    overallRTDA += ObjRTDA(rtdaVec_[i]);
                double res = overallRTDA * weightInMpRTDA;
                if (considerSensorFusion == 0)
                {
                    // Optmize RTDA: obj = max_RTs + max_DAs + overallRTDA * weightInMpRTDA
                    res += ReadObj();
                }
                else
                {
                    // only used in RTSS21IC experiment
                    // Optimize Sensor Fusion: obj = overallRTDA * someWeight + overallSensorFusion * someWeight +
                    // {Barrier(max(RT)) * w_punish + Barrier(max(DA)) * w_punish}**for_every_chain + Barrier(max(SensorFusion)) * w_punish
                    double sfOverall = sfVec_.sum();
                    res += sfOverall * weightInMpSf;
                    res += ObjBarrier();
                }
                return res;
            }
            double ObjBarrier()
            {
                if (considerSensorFusion == 0)
                    return ReadObj();
                else
                {
                    double error = 0;
                    if (sfVec_.rows() > 0)
                        error = Barrier(sensorFusionTolerance - sfVec_.maxCoeff()) * weightInMpSfPunish;
                    for (uint i = 0; i < dagTasks_.chains_.size(); i++)
                    {
                        error += Barrier(FreshTol - maxRtda_[i].reactionTime) * weightInMpRTDAPunish + Barrier(FreshTol - maxRtda_[i].dataAge) * weightInMpRTDAPunish;
                    }
                    return error;
                }
            }
        };

        std::vector<int> FindLongestChainJobIndex(IterationStatus &status)
        {
            std::vector<int> index(status.rtdaVec_.size(), 0);
            for (uint i = 0; i < status.rtdaVec_.size(); i++) // for each chain
            {
                std::vector<RTDA> &rtdaVec = status.rtdaVec_[i];
                auto ite = std::max_element(rtdaVec.begin(), rtdaVec.end(), [](RTDA r1, RTDA r2)
                                            { return ObjRTDA(r1) < ObjRTDA(r2); });
                index[i] = std::distance(rtdaVec.begin(), ite);
            }
            return index;
        }
        int infeasibleCount = 0;
        bool MakeProgress(IterationStatus &statusPrev, IterationStatus &statusCurr)
        {
            if (!statusCurr.schedulable_)
            {
                if (debugMode == 1)
                {
                    TaskSetInfoDerived tasksInfo(statusCurr.dagTasks_.tasks);
                    std::cout << "Infeasible schedule #:" << infeasibleCount++ << std::endl;
                    // PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                    statusCurr.jobOrder_.print();
                }
                return false;
            }
            if (statusCurr.objWeighted_ < statusPrev.objWeighted_)
                return true;
            return false;
        }

        bool WhetherSkipInsertStart(JobCEC &jobRelocate, LLint startP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
        {
            if (startP > 0)
            {
                TimeInstance instancePrev = jobOrderCurr.instanceOrder_[startP - 1];
                // jP.ActivationTime <= jR.start <= jR.deadline - jR.executionTime
                if (GetActivationTime(instancePrev.job, tasksInfo) > GetDeadline(jobRelocate, tasksInfo) - tasksInfo.tasks[jobRelocate.taskId].executionTime)
                    return true;
            }
            if (startP < tasksInfo.length * 2 - 1)
            {
                TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[startP + 1];
                //  jR.ActivationTime <= jR.start <= nextJ.Deadline
                double nextInstanceLeastFinishTime = GetDeadline(instanceAfter.job, tasksInfo);
                if (instanceAfter.type == 's')
                    nextInstanceLeastFinishTime -= tasksInfo.tasks[instanceAfter.job.taskId].executionTime;
                if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                    return true;
            }
            return false;
        }
        bool WhetherSkipInsertFinish(JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurr)
        {
            if (finishP > 0)
            {
                TimeInstance instancePrev = jobOrderCurr.instanceOrder_[finishP - 1];
                // jP.ActivationTime <= jR.finish <= jR.deadline
                if (GetActivationTime(instancePrev.job, tasksInfo) > GetDeadline(jobRelocate, tasksInfo))
                    return true;
            }
            if (finishP < tasksInfo.length * 2 - 1)
            {
                TimeInstance instanceAfter = jobOrderCurr.instanceOrder_[finishP + 1];
                //  jR.ActivationTime <= jR.finish <= nextJ.Deadline
                double nextInstanceLeastFinishTime = GetDeadline(instanceAfter.job, tasksInfo);
                if (instanceAfter.type == 's')
                    nextInstanceLeastFinishTime -= tasksInfo.tasks[instanceAfter.job.taskId].executionTime;
                if (GetActivationTime(jobRelocate, tasksInfo) > nextInstanceLeastFinishTime)
                    return true;
            }
            return false;
        }
        bool WhetherStartFinishTooLong(double &accumLengthMin, JobCEC &jobRelocate, LLint finishP, const TaskSetInfoDerived &tasksInfo, const SFOrder &jobOrderCurrForStart, LLint startP)
        {
            if (accumLengthMin >= tasksInfo.tasks[jobRelocate.taskId].executionTime)
                return true;
            TimeInstance jobPrevInsertInst = jobOrderCurrForStart.at(finishP);
            if (jobPrevInsertInst.type == 'f') // && jobOrderCurrForStart.GetJobStartInstancePosition(jobPrevInsertInst.job) > startP
                accumLengthMin += tasksInfo.tasks[jobPrevInsertInst.job.taskId].executionTime;
            return false;
        }
        ScheduleResult ScheduleDAGModel(DAG_Model &dagTasks, int processorNum = coreNumberAva,
                                        boost::optional<ScheduleResult &> resOrderOptWithoutScheduleOpt = boost::none)
        {
            // srand(RandomDrawWeightMaxLoop);
            if (dagTasks.chains_.size() == 0)
                CoutWarning("No chain is provided for the given dag!");

            TaskSet &tasks = dagTasks.tasks;
            TaskSetInfoDerived tasksInfo(tasks);
            // VectorDynamic initialSTV = SFOrderScheduling(dagTasks, tasksInfo, processorNum);
            VectorDynamic initialSTV = ListSchedulingLFTPA(dagTasks, tasksInfo, processorNum);
            if (debugMode == 1)
            {
                std::cout << "Initial schedule: " << std::endl;
                PrintSchedule(tasksInfo, initialSTV);
                std::cout << initialSTV << std::endl;
            }

            SFOrder jobOrderRef(tasksInfo, initialSTV);
            if (debugMode == 1)
            {
                std::cout << "Initial SF order: " << std::endl;
                jobOrderRef.print();
            }
            IterationStatus statusPrev(dagTasks, tasksInfo, jobOrderRef, processorNum);
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Initial schedule is not schedulable!!!");
            }
            bool foundOptimal = false;
            bool findNewUpdate = true;
            LLint countMakeProgress = 0;
            LLint countIterationStatus = 0;

            if (statusPrev.ObjBarrier() == 0)
                foundOptimal = true;
            auto ExamAndApplyUpdate = [&](SFOrder &jobOrderCurr)
            {
                BeginTimer("IterationStatusCreate");
                IterationStatus statusCurr(dagTasks, tasksInfo, jobOrderCurr, processorNum);
                EndTimer("IterationStatusCreate");

                countIterationStatus++;
                if (MakeProgress(statusPrev, statusCurr))
                {
                    findNewUpdate = true;
                    statusPrev = statusCurr;
                    if (debugMode == 1)
                    {
                        std::cout << "Make progress!" << std::endl;
                        PrintSchedule(tasksInfo, statusCurr.startTimeVector_);
                    }
                    countMakeProgress++;
                    if (statusCurr.ObjBarrier() == 0)
                        foundOptimal = true;
                }
            };

            auto start_time = std::chrono::system_clock::now();
            auto curr_time = std::chrono::system_clock::now();
            int64_t time_limit_in_seconds = makeProgressTimeLimit;
            if (time_limit_in_seconds < 0)
            {
                time_limit_in_seconds = INT64_MAX;
            }
            bool time_out_flag = false;

            auto CheckTimeOut = [&]()
            {
                curr_time = std::chrono::system_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count() >= time_limit_in_seconds)
                {
                    std::cout << "\nTime out when running OptimizeOrder. Maximum time is " << time_limit_in_seconds << " seconds.\n\n";
                    time_out_flag = true;
                    return true;
                }
                return false;
            };
            // int jobWithMaxChain = FindLongestChainJobIndex(statusPrev)[0];
            int jobWithMaxChain = 0;

            LLint countOutermostWhileLoop = 0;
            while (findNewUpdate)
            {
                countOutermostWhileLoop++;
                if (time_out_flag || foundOptimal)
                    break;

                findNewUpdate = false;

                // search the tasks related to task chain at first
                std::vector<int> taskIdSet = GetTaskIdWithChainOrder(dagTasks);
                BeginTimer("inner_for_job");
                for (int i : taskIdSet)
                    for (LLint j = jobWithMaxChain; j < jobWithMaxChain + tasksInfo.sizeOfVariables[i]; j++)
                    {
                        if (time_out_flag || foundOptimal)
                            break;
                        JobCEC jobRelocate(i, j % tasksInfo.sizeOfVariables[i]);
                        LLint prevJobIndex = 0, nextJobIndex = static_cast<LLint>(statusPrev.jobOrder_.size() - 1);
                        if (jobRelocate.jobId > 0)
                        {
                            JobCEC prevJob(jobRelocate.taskId, jobRelocate.jobId - 1);
                            prevJobIndex = statusPrev.jobOrder_.GetJobFinishInstancePosition(prevJob);
                        }
                        if (jobRelocate.jobId < tasksInfo.sizeOfVariables[jobRelocate.taskId] - 1)
                        {
                            JobCEC nextJob(jobRelocate.taskId, jobRelocate.jobId + 1);
                            nextJobIndex = std::min(statusPrev.jobOrder_.GetJobStartInstancePosition(nextJob) + 1, nextJobIndex); // actually, I'm not sure why do we need this "+1", but it doesn't hurt to search for a few more
                        }

                        for (LLint startP = prevJobIndex; startP < nextJobIndex; startP++)
                        {
                            BeginTimer("inner_for_start");
                            if (time_out_flag || foundOptimal)
                                break;
                            if (statusPrev.jobOrder_[startP].job.taskId == jobRelocate.taskId && statusPrev.jobOrder_[startP].job.jobId > jobRelocate.jobId)
                                break;

                            SFOrder jobOrderCurrForStart = statusPrev.jobOrder_;
                            jobOrderCurrForStart.RemoveJob(jobRelocate);
                            if (WhetherSkipInsertStart(jobRelocate, startP, tasksInfo, jobOrderCurrForStart))
                                continue;

                            jobOrderCurrForStart.InsertStart(jobRelocate, startP); // must insert start first
                            double accumLengthMin = 0;
                            for (LLint finishP = startP + 1; finishP < nextJobIndex + 1; finishP++)
                            {
                                if (CheckTimeOut())
                                    break;
                                if (WhetherSkipInsertFinish(jobRelocate, finishP, tasksInfo, statusPrev.jobOrder_))
                                    continue;
                                if (WhetherStartFinishTooLong(accumLengthMin, jobRelocate, finishP, tasksInfo, jobOrderCurrForStart, startP))
                                    break;

                                BeginTimer("SFOrderConstructor");
                                SFOrder jobOrderCurrForFinish = jobOrderCurrForStart;
                                EndTimer("SFOrderConstructor");
                                jobOrderCurrForFinish.InsertFinish(jobRelocate, finishP);
                                // if (debugMode == 1)
                                //     jobOrderCurrForFinish.print();

                                ExamAndApplyUpdate(jobOrderCurrForFinish);
                                if (foundOptimal)
                                    break;
                            }
                            EndTimer("inner_for_start");
                        }
                    }
                // std::cout << "Finish one big while loop!" << std::endl;
                EndTimer("inner_for_job");
            }
            if (!statusPrev.schedulable_)
            {
                CoutWarning("Optimize SFOrder return with unschedulable result!");
            }

            ScheduleResult scheduleRes{statusPrev.jobOrder_, statusPrev.startTimeVector_, statusPrev.schedulable_, statusPrev.ReadObj(), statusPrev.processorJobVec_};
            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
            scheduleRes.objWeighted_ = statusPrev.objWeighted_;

            if (doScheduleOptimization && !foundOptimal)
            {
                if (!considerSensorFusion || !scheduleRes.schedulable_)
                {
                    ScheduleOptimizer schedule_optimizer = ScheduleOptimizer();
                    ScheduleResult result_after_optimization;
                    if (considerSensorFusion)
                    {
                        schedule_optimizer.OptimizeObjWeighted(dagTasks, scheduleRes);
                        result_after_optimization = schedule_optimizer.getOptimizedResult();
                        if (result_after_optimization.objWeighted_ < scheduleRes.objWeighted_)
                        {
                            scheduleRes = result_after_optimization;
                            std::vector<RTDA> rtda_vector;
                            for (auto chain : dagTasks.chains_)
                            {
                                auto res = GetRTDAFromSingleJob(tasksInfo, chain, scheduleRes.startTimeVector_);
                                RTDA resM = GetMaxRTDA(res);
                                rtda_vector.push_back(resM);
                            }
                            scheduleRes.obj_ = ObjRTDA(rtda_vector);
                        }
                    }
                    else
                    {
                        schedule_optimizer.Optimize(dagTasks, scheduleRes);
                        scheduleRes = schedule_optimizer.getOptimizedResult();
                    }
                }
            }

            scheduleRes.schedulable_ = ExamAll_Feasibility(dagTasks, tasksInfo, scheduleRes.startTimeVector_, scheduleRes.processorJobVec_, processorNum, sensorFusionTolerance, FreshTol);
            std::cout << "Outermost while loop count: " << countOutermostWhileLoop << std::endl;
            std::cout << "Make progress count: " << countMakeProgress << std::endl;
            std::cout << "Candidate Iteration Status count: " << countIterationStatus << std::endl;
            scheduleRes.countOutermostWhileLoop_ = countOutermostWhileLoop;
            scheduleRes.countMakeProgress_ = countMakeProgress;
            scheduleRes.countIterationStatus_ = countIterationStatus;
            return scheduleRes;
        }
    } // namespace OptimizeSF
} // namespace OrderOptDAG_SPACE
