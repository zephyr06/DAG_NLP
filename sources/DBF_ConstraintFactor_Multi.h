#pragma once
#include "BaseSchedulingFactor.h"
#include "Interval.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    class DBF_ConstraintFactor_Multi : public BaseSchedulingFactor
    {
    public:
        DBF_ConstraintFactor_Multi(Key key, TaskSetInfoDerived &tasksInfo,
                                   EliminationForest &forestInfo, LLint errorDimension,
                                   SharedNoiseModel model)
            : BaseSchedulingFactor(key, tasksInfo, forestInfo, errorDimension, model)
        {
        }

        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {
            BeginTimer("DBF_All");
            if (H)
            {
                // BeginTimer("DBF_H");
                // if (numericalJaobian)
                // {
                *H = NumericalDerivativeDynamicUpperDBF(f, startTimeVector, deltaOptimizer, errorDimension);
                // }
                // else
                //     *H = DBFJacobian(f, startTimeVector, deltaOptimizer, errorDimension);
                // EndTimer("DBF_H");
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DBF_ConstraintFactor_Multi is " << endl
                         << *H << endl;
                    cout << Color::green << "The input startTimeVector of DBF is " << startTimeVector << Color::def << endl;
                    cout << "The error vector of DBF is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
                if ((*H).norm() <= zeroJacobianDetectTol && f(startTimeVector).norm() != 0)
                {
                    CoutWarning("DBF factor: 0 Jacobian while non-zero error found!");
                }

                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    auto sth = NumericalDerivativeDynamicUpperDBF(f, startTimeVector, deltaOptimizer, errorDimension);
                    cout << "The Jacobian matrix of DBF_ConstraintFactor_Multi is " << endl
                         << sth << endl;
                    cout << Color::green << "The input startTimeVector of DBF is " << startTimeVector << Color::def << endl;
                    cout << "The error vector of DBF is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
                if (debugMode == 3)
                {
                    cout << "The error vector of DBF is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }
            EndTimer("DBF_All");
            return f(startTimeVector);
        }

        /**
         * @brief for error evaluation; this returns a VectorDynamic, 
         * which is the merged version of all DBF error for each processor
         * 
         */
        struct SF_Time_Instance
        {
            char type;
            double time;
            int coreRequire;
        };
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic res;
            res.resize(errorDimension, 1);

            VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                                   forestInfo);

            int indexPro = 0;

            for (auto itr = tasksInfo.processorTaskSet.begin(); itr != tasksInfo.processorTaskSet.end(); itr++)
            {
                double errorCurrProc = 0;
                vector<SF_Time_Instance> SF_Seq;
                SF_Seq.reserve(startTimeVector.size() * 2);
                for (int taskId : tasksInfo.processorTaskSet.at(itr->first))
                {
                    for (int j = 0; j < tasksInfo.sizeOfVariables[taskId]; j++)
                    {
                        SF_Time_Instance inst{'s',
                                              startTimeVector(IndexTran_Instance2Overall(taskId,
                                                                                         j,
                                                                                         tasksInfo.sizeOfVariables),
                                                              0),
                                              tasksInfo.tasks[taskId].coreRequire};
                        SF_Seq.push_back(inst);
                        inst = {'f', inst.time + tasksInfo.tasks[taskId].executionTime, tasksInfo.tasks[taskId].coreRequire};
                        SF_Seq.push_back(inst);
                    }
                }
                std::sort(SF_Seq.begin(), SF_Seq.end(),
                          [](SF_Time_Instance &a, SF_Time_Instance &b) -> bool
                          {
                              return a.time < b.time;
                          });
                double loadCurr = SF_Seq[0].coreRequire;
                double timePrev = SF_Seq[0].time;
                for (LLint i = 1; i < LLint(SF_Seq.size()); i++)
                {
                    if (loadCurr > coreNumberAva)
                    {
                        errorCurrProc += (SF_Seq[i].time - timePrev) * (loadCurr - coreNumberAva);
                    }
                    if (SF_Seq[i].type == 's')
                    {
                        loadCurr += SF_Seq[i].coreRequire;
                    }
                    else if (SF_Seq[i].type == 'f')
                    {
                        loadCurr -= SF_Seq[i].coreRequire;
                    }
                    else
                        CoutError("Not recognized type in SF_Seq");
                    timePrev = SF_Seq[i].time;
                }
                res(indexPro++, 0) = errorCurrProc;
                //     double errorCurrProcessor = 0;
                //     vector<int> taskSetCurr = processorTaskSet.at(itr->first);
                //     // //demand bound function
                //     for (int index_i = 0; index_i < static_cast<int>(taskSetCurr.size()); index_i++)
                //     {
                //         int i = taskSetCurr[index_i];
                //         for (LLint instance_i = 0; instance_i < sizeOfVariables[i]; instance_i++)
                //         {
                //             double startTime_i = ExtractVariable(startTimeVector, sizeOfVariables, i, instance_i);
                //             for (int index_j = index_i; index_j < static_cast<int>(taskSetCurr.size()); index_j++)
                //             {
                //                 int j = taskSetCurr[index_j];
                //                 for (LLint instance_j = 0; instance_j < sizeOfVariables[j]; instance_j++)
                //                 {
                //                     double sumIJK = 0;
                //                     double startTime_j = ExtractVariable(startTimeVector, sizeOfVariables, j, instance_j);
                //                     if (startTime_i <= startTime_j &&
                //                         startTime_i + tasks[i].executionTime <= startTime_j + tasks[j].executionTime)
                //                     {
                //                         for (int index_k = 0; index_k < static_cast<int>(taskSetCurr.size()); index_k++)
                //                         {
                //                             int k = taskSetCurr[index_k];
                //                             for (LLint instance_k = 0; instance_k < sizeOfVariables[k]; instance_k++)
                //                             {
                //                                 double startTime_k = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_k);
                //                                 sumIJK += ComputationTime_IJK(startTime_i, tasks[i], startTime_j, tasks[j], startTime_k, tasks[k]) * tasks[k].coreRequire;
                //                             }
                //                         }
                //                         double valueT = Barrier((startTime_j + tasks[j].executionTime - startTime_i) * coreNumberAva - sumIJK + 0);
                //                         errorCurrProcessor += valueT;
                //                     }
                //                     else
                //                     {
                //                         continue;
                //                     }
                //                 }
                //             }
                //         }
            }

            return res;
        };

        /**
         * @brief this version of Jacobian estimation fix the Vanishing gradient problem;
         * 
         * this program will identify indexes that have vanishing gradient issues, 
         * i.e., an interval is fullly covered by another interval without bound overlap,
         * then apply a bigger, but tight deltaOptimizer for these special indexes to avoid vanishing gradient; 
         * (probably use binary search to find it)
         * other indexes are handled normally by provided deltaOptimizer;
         * 
         * As for identified as overlap index, increasing deltaOptimizer is only applied when it has a real zero
         *  gradient at this point; this is a necessary condition
         * 
         * @param h 
         * @param x 
         * @param deltaOptimizer 
         * @param mOfJacobian 
         * @return MatrixDynamic 
         */
        MatrixDynamic NumericalDerivativeDynamicUpperDBF(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                                         const VectorDynamic &x, double deltaOptimizer,
                                                         int mOfJacobian) const
        {

            return NumericalDerivativeDynamicUpper(h, x, deltaOptimizer, mOfJacobian);
        }
    };
}