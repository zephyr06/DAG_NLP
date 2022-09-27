
#pragma once

#include "sources/Utils/DeclareDAG.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Factors/DDL_ConstraintFactor.h"
namespace DAG_SPACE
{

    // move start time of small interval to the end of large interval
    // TODO:  first try begin and end, if both fails, they randomly generate a start time within the feasible region, and keep excluding regions where gradient vanish is possible. Algorithm terminates if there is no such possible space to generate start time.
    enum RelocationMethod
    {
        EndOfLongInterval,
        BeginOfLongInterval,
        EndOfShortInterval,
        BeginOfShortInterval,
        RandomOfInterval
    };
    struct GradientVanishPairs
    {
        std::vector<gtsam::KeyVector> vanishPairs_;
        void add(gtsam::KeyVector &keys)
        {
            vanishPairs_.push_back(keys);
        }
        size_t size() const
        {
            return vanishPairs_.size();
        }
    };
    struct GradientVanishDetectResult
    {
        bool whetherExist;
        struct GradientVanishPairs gradientVanishPairs;
        VectorDynamic startTimeVectorAfterRelocate;
    };
    bool operator==(GradientVanishPairs &p1, GradientVanishPairs &p2)
    {
        if (p1.size() != p2.size())
        {
            return false;
        }
        else
        {
            for (size_t i = 0; i < p1.size(); i++)
            {
                gtsam::KeyVector &vec1 = p1.vanishPairs_[i];
                gtsam::KeyVector &vec2 = p2.vanishPairs_[i];
                if (vec1.size() != vec2.size())
                {
                    return false;
                }
                for (size_t j = 0; j < vec1.size(); j++)
                {
                    if (vec1[j] != vec2[j])
                        return false;
                }
            }
        }
        return true;
    }
    bool operator!=(GradientVanishPairs &p1, GradientVanishPairs &p2)
    {
        return !(p1 == p2);
    }

    /********************************** Below are methods **********************************/
    inline RelocationMethod IncrementRelocationMethod(RelocationMethod x)
    {
        if (x != RandomOfInterval)
            return static_cast<RelocationMethod>(static_cast<int>(x) + 1);
        else
        {
            return EndOfLongInterval;
        }
    }

    // TODO: test this function
    // try 4 relocate methods, use the first method that delivers feasible solution, or anyone if none is feasible; This function may perform in-place modification on stvRes
    bool PickRelocateMethodAndMove(TaskSetInfoDerived &tasksInfo, int smallTaskIndex, int smallJobIndex, int largeTaskIndex, int largeJobIndex, LLint indexSmallInSTV, LLint indexLargeInSTV, VectorDynamic &startTimeVector)
    {
        NonlinearFactorGraph graph;
        AddDDL_JobFactor(graph, tasksInfo, smallTaskIndex, smallJobIndex);
        AddDDL_JobFactor(graph, tasksInfo, largeTaskIndex, largeJobIndex);
        Values stvValues = GenerateInitialFG(startTimeVector, tasksInfo);
        double errorBase = graph.error(stvValues);

        auto ExamUpdate = [&](int taskId, int jobId, LLint indexInSTV, double starTime) -> bool
        {
            stvValues.update(GenerateKey(taskId, jobId), GenerateVectorDynamic1D(starTime));
            if (graph.error(stvValues) <= errorBase)
            {
                startTimeVector(indexInSTV) = starTime;
                return true;
            }
            else
            {
                stvValues.update(GenerateKey(taskId, jobId), GenerateVectorDynamic1D(startTimeVector(indexInSTV)));
                return false;
            }
        };

        // move small task to beginning of large task
        double starTime = startTimeVector(indexLargeInSTV) - tasksInfo.tasks[smallTaskIndex].executionTime;
        if (ExamUpdate(smallTaskIndex, smallJobIndex, indexSmallInSTV, starTime))
            return true;

        // move small task to ending of large task
        starTime = startTimeVector(indexLargeInSTV) + tasksInfo.tasks[largeTaskIndex].executionTime;
        if (ExamUpdate(smallTaskIndex, smallJobIndex, indexSmallInSTV, starTime))
            return true;

        // move large task to beginning of small task
        starTime = startTimeVector(indexSmallInSTV) - tasksInfo.tasks[largeTaskIndex].executionTime;
        if (ExamUpdate(largeTaskIndex, largeJobIndex, indexLargeInSTV, starTime))
            return true;

        // move large task to ending of small task
        starTime = startTimeVector(indexSmallInSTV) + tasksInfo.tasks[smallTaskIndex].executionTime;
        if (ExamUpdate(largeTaskIndex, largeJobIndex, indexLargeInSTV, starTime))
            return true;
        return false;
    }

    VectorDynamic FindEmptyPosition(TaskSetInfoDerived &tasksInfo, gtsam::Symbol smallJobKey, gtsam::Symbol largeJobKey, VectorDynamic &startTimeVector, RelocationMethod relocateMethod = EndOfLongInterval)
    {
        VectorDynamic stvRes = startTimeVector;
        int smallTaskIndex, smallJobIndex, largeTaskIndex, largeJobIndex;
        std::tie(smallTaskIndex, smallJobIndex) = AnalyzeKey(smallJobKey);
        std::tie(largeTaskIndex, largeJobIndex) = AnalyzeKey(largeJobKey);
        LLint indexSmallInSTV = IndexTran_Instance2Overall(smallTaskIndex, smallJobIndex, tasksInfo.sizeOfVariables);
        LLint indexLargeInSTV = IndexTran_Instance2Overall(largeTaskIndex, largeJobIndex, tasksInfo.sizeOfVariables);

        bool whetherSuccess = PickRelocateMethodAndMove(tasksInfo, smallTaskIndex, smallJobIndex, largeTaskIndex, largeJobIndex, indexSmallInSTV, indexLargeInSTV, stvRes);
        if (!whetherSuccess)
        {
            switch (relocateMethod)
            {
            case BeginOfLongInterval:
                // put it at the begining of large task
                stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) - tasksInfo.tasks[smallTaskIndex].executionTime;
                break;
            case EndOfLongInterval:
                // put it at the end of large task
                stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) + tasksInfo.tasks[largeTaskIndex].executionTime;
                break;
            case BeginOfShortInterval:
                stvRes(indexLargeInSTV) = startTimeVector(indexSmallInSTV) - tasksInfo.tasks[largeTaskIndex].executionTime;
                break;
            case EndOfShortInterval:
                stvRes(indexLargeInSTV) = startTimeVector(indexSmallInSTV) + tasksInfo.tasks[smallTaskIndex].executionTime;
                break;
            case RandomOfInterval:
                stvRes(indexSmallInSTV) = startTimeVector(indexLargeInSTV) + tasksInfo.tasks[smallTaskIndex].executionTime;
                break;
            }
        }

        return stvRes;
    }

    // move some variables that suffer from zero gradient issue around
    GradientVanishDetectResult RelocateIncludedInterval(TaskSetInfoDerived &tasksInfo,
                                                        NonlinearFactorGraph &graph,
                                                        VectorDynamic startTimeVector, RelocationMethod relocateMethod = EndOfLongInterval)
    {

        Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
        bool whetherRelocate = false;
        GradientVanishPairs gradientVanishPairs;
        for (auto ite = graph.begin(); ite != graph.end(); ite++)
        {
            double err = ite->get()->error(initialEstimateFG);
            if (err == 0)
            {
                continue;
            }
            double deltaOptimizerRef = deltaOptimizer;
            deltaOptimizer = 1e-6;
            auto pp = ite->get()->linearize(initialEstimateFG)->jacobian();
            deltaOptimizer = deltaOptimizerRef;
            MatrixDynamic jacobian = pp.first;
            if (jacobian.norm() / pow(err * 2, 0.5) < zeroJacobianDetectTol)
            {
                std::cout << "Vanish DBF factor: " << std::endl;
                ite->get()->printKeys();
                auto keys = ite->get()->keys();
                if (keys.size() == 2)
                {
                    gradientVanishPairs.add(keys);
                    int task0Index, job0Index, task1Index, job1Index;
                    std::tie(task0Index, job0Index) = AnalyzeKey(keys[0]);
                    std::tie(task1Index, job1Index) = AnalyzeKey(keys[1]);
                    if (tasksInfo.tasks[task0Index].executionTime < tasksInfo.tasks[task1Index].executionTime)
                    {
                        startTimeVector = FindEmptyPosition(tasksInfo, keys[0], keys[1], startTimeVector, relocateMethod);

                        initialEstimateFG.update(keys[0], GenerateVectorDynamic1D(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, task0Index, job0Index)));
                    }
                    else
                    {
                        startTimeVector = FindEmptyPosition(tasksInfo, keys[1], keys[0], startTimeVector, relocateMethod);
                        initialEstimateFG.update(keys[1], GenerateVectorDynamic1D(ExtractVariable(startTimeVector, tasksInfo.sizeOfVariables, task1Index, job1Index)));
                    }
                    whetherRelocate = true;
                }
                else
                {
                    CoutWarning("Vanish gradient of non-DBF type is found, please add related relocation method!");
                    // double err = ite->get()->error(initialEstimateFG);
                    // int a = 1;
                }
            }
        }
        return {whetherRelocate, gradientVanishPairs, startTimeVector};
    }
} // namespace DAG_SPACE