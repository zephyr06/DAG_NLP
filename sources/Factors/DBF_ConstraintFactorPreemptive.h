#pragma once

#include "sources/Factors/Interval.h"
#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/RegularTasks.h"

namespace OrderOptDAG_SPACE
{

    using namespace RegularTaskSystem;

    // this function finds the job instances that could not possibly contribute into cijk or cjik by examing the start&finish time
    std::vector<gtsam::Symbol> FindPossibleOverlapKeys(int iTask, int iJob, int jTask, int jJob, TaskSetInfoDerived &tasksInfo)
    {
        std::vector<gtsam::Symbol> keys;
        keys.push_back(GenerateKey(iTask, iJob));
        // it's okay if two keys are the same, we leave the error function to handle this special case
        keys.push_back(GenerateKey(jTask, jJob));

        double start = std::min(tasksInfo.tasks[iTask].period * iJob, tasksInfo.tasks[jTask].period * jJob);
        double endInner = std::max(tasksInfo.tasks[iTask].period * (iJob + 1), tasksInfo.tasks[jTask].period * (jJob + 1));

        // go through all the job instances and include those that possibly contribute into c_ijk
        for (int kTask = 0; kTask < tasksInfo.N; kTask++)
        {
            for (int kJob = 0; kJob < tasksInfo.sizeOfVariables[kTask]; kJob++)
            {
                if ((kTask == iTask && kJob == iJob) || (kTask == jTask && kJob == jJob))
                {
                    continue;
                }
                double startK = tasksInfo.tasks[kTask].period * kJob;
                double endK = tasksInfo.tasks[kTask].period * (1 + kJob);

                if (endK <= start)
                { // TODO: improve with binary search
                    continue;
                }
                else if (startK >= endInner)
                {
                    break;
                }
                else
                {
                    keys.push_back(GenerateKey(kTask, kJob));
                }
            }
        }
        return keys;
    }

    VectorDynamic DBF_PreemptError(const Values &x, const std::vector<gtsam::Symbol> &keys, const TaskSetInfoDerived &tasksInfo)
    {
        double si = std::min(x.at<VectorDynamic>(keys[0])(0), x.at<VectorDynamic>(keys[1])(0));
        double fj = std::max(x.at<VectorDynamic>(keys[0])(1), x.at<VectorDynamic>(keys[1])(1));
        if (fj < si)
        {
            return GenerateVectorDynamic1D(0);
        }

        double slack = fj - si;
        // avoid deduct the interval itself twice
        int beginIndex = 0;
        if (keys[0] == keys[1])
            beginIndex++;
        for (size_t i = beginIndex; i < keys.size(); i++)
        {
            double sk = x.at<VectorDynamic>(keys[i])(0);
            double fk = x.at<VectorDynamic>(keys[i])(1);
            if (si <= sk && fk <= fj)
            {
                int taskIndex = AnalyzeKey(keys[i]).first;
                slack -= tasksInfo.tasks[taskIndex].executionTime;
            }
        }
        if (debugMode > 0)
        {
            auto x1 = x.at<VectorDynamic>(keys[0]);
            auto x2 = x.at<VectorDynamic>(keys[1]);
            // int a = 1;
        }
        return GenerateVectorDynamic1D(Barrier(slack));
    }

    void AddDBFPreempt_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
    {
        auto model1 = noiseModel::Isotropic::Sigma(1, noiseModelSigma);

        // find all the job pairs, n(n+1)/2 in total, including self-pairs
        // All the job pairs must be kept to guarantee DBF check safe
        for (int iTask = 0; iTask < tasksInfo.N; iTask++)
        {
            for (int iJob = 0; iJob < int(tasksInfo.sizeOfVariables[iTask]); iJob++)
            {
                // double start = tasksInfo.tasks[iTask].period * iJob;
                // double end = tasksInfo.tasks[iTask].period * (iJob + 1);

                for (int jTask = iTask; jTask < tasksInfo.N; jTask++)
                {
                    for (int jJob = iJob; jJob < int(tasksInfo.sizeOfVariables[jTask]); jJob++)
                    {
                        // if (iTask == jTask && iJob == jJob)
                        // {
                        //     continue;
                        // }
                        // double startInner = tasksInfo.tasks[jTask].period * jJob;
                        // double endInner = tasksInfo.tasks[jTask].period * (jJob + 1);

                        std::vector<gtsam::Symbol> keys = FindPossibleOverlapKeys(iTask, iJob, jTask, jJob, tasksInfo);

                        // error function for DBF(MultiKey) factor
                        LambdaMultiKey f = [keys, tasksInfo](const Values &x)
                        {
                            return DBF_PreemptError(x, keys, tasksInfo);
                        };
                        graph.emplace_shared<MultiKeyFactor>(keys, f, 1, model1);
                    }
                }
            }
        }
    }
}