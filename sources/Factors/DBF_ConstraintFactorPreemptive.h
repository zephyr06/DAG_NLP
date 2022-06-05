#include "sources/Factors/MultiKeyFactor.h"
#include "sources/TaskModel/RegularTasks.h"

namespace DAG_SPACE
{

    using namespace RegularTaskSystem;

    void AddDBFPreempt_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
    {
        auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);

        for (int iTask = 0; iTask < tasksInfo.N; iTask++)
        {
            for (int iJob = 0; iJob < int(tasksInfo.sizeOfVariables[iTask]); iJob++)
            {
                LLint index_overall = IndexTran_Instance2Overall(iTask, iJob, tasksInfo.sizeOfVariables);
                Symbol key = GenerateKey(iTask, iJob);
                double start = tasksInfo.tasks[iTask].period * iJob;
                double end = tasksInfo.tasks[iTask].period * (iJob + 1);

                for (int jTask = iTask + 1; jTask < tasksInfo.N; jTask++)
                {
                    for (int jJob = 0; jJob < int(tasksInfo.sizeOfVariables[jTask]); jJob++)
                    {
                        double startInner = tasksInfo.tasks[jTask].period * jJob;
                        double endInner = tasksInfo.tasks[jTask].period * (jJob + 1);
                        // only preceed if [start, end] and [startInner, endInner] are included within
                        // [start endInner]
                        // we assume f_i >= s_i + c_i
                        // todo: TEST DBF_ConstraintFactorPreemptive
                        if (!(end <= endInner && start <= startInner))
                        {
                            continue;
                        }
                        std::vector<gtsam::Symbol> keys;
                        keys.push_back(GenerateKey(iTask, iJob));
                        keys.push_back(GenerateKey(jTask, jJob));

                        // go through all the job instances and include those that possibly contribute into c_ijk
                        for (int kTask = 0; kTask < tasksInfo.N; kTask++)
                        {
                            if (kTask == iTask || kTask == iJob)
                            {
                                continue;
                            }
                            for (int kJob = 0; kJob < tasksInfo.sizeOfVariables[kTask]; kJob++)
                            {
                                ;
                            }
                        }

                        // LLint index_overall_inner = IndexTran_Instance2Overall(jTask, jJob, tasksInfo.sizeOfVariables);
                        // Symbol key_inner = GenerateKey(jTask, jJob);
                        // Interval v1 = CreateSingleInterval(index_overall, 0.0,
                        //                                    tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        // Interval v2 = CreateSingleInterval(index_overall_inner, 0.0,
                        //                                    tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        // NormalErrorFunction2D DBF2D =
                        //     [v1, v2](VectorDynamic x1, VectorDynamic x2)
                        // {
                        //     BeginTimer("DBF_Lambda");
                        //     VectorDynamic res = x1;
                        //     Interval v11 = v1;
                        //     Interval v22 = v2;
                        //     v11.start = x1(0, 0);
                        //     v22.start = x2(0, 0);

                        //     res << Overlap(v11, v22);
                        //     EndTimer("DBF_Lambda");
                        //     return res;
                        // };
                        // // this factor is explained as: variable * 1 <= tasks[iTask].deadline + iTask * tasks[iTask].period
                        // graph.emplace_shared<InequalityFactor2D>(key, key_inner, DBF2D, model);
                    }
                }
            }
        }
    }
}