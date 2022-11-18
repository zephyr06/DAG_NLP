#pragma once
#include "DeclareDAG.h"

#include "sources/TaskModel/RegularTasks.h"
namespace RTSS21IC_NLP
{

    namespace DAG_SPACE
    {
        using namespace RegularTaskSystem;
        class DAG_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
        {
        public:
            OrderOptDAG_SPACE::DAG_Model dagTasks;
            TaskSet tasks;
           std::vector<LLint> sizeOfVariables;
            int N;
            LLint errorDimension;
            LLint length;
           std::vector<bool> maskForEliminate;
            LLint lengthCompressed;
            MAP_Index2Data mapIndex;
            std::unordered_map<LLint, LLint> mapIndex_True2Compress;

            DAG_ConstraintFactor(Key key, OrderOptDAG_SPACE::DAG_Model &dagTasks,std::vector<LLint> sizeOfVariables, LLint errorDimension,
                                 MAP_Index2Data &mapIndex, const std::vector<bool> &maskForEliminate,
                                 SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                           dagTasks(dagTasks),
                                                           tasks(dagTasks.tasks), sizeOfVariables(sizeOfVariables),
                                                           N(tasks.size()), errorDimension(errorDimension),
                                                           maskForEliminate(maskForEliminate),
                                                           mapIndex(mapIndex)
            {
                length = 0;

                for (int i = 0; i < N; i++)
                {
                    length += sizeOfVariables[i];
                }
                lengthCompressed = 0;
                for (LLint i = 0; i < length; i++)
                {
                    if (maskForEliminate[i] == false)
                        lengthCompressed++;
                }
                mapIndex_True2Compress = MapIndex_True2Compress(maskForEliminate);
            }
            boost::function<Matrix(const VectorDynamic &)> f =
                [this](const VectorDynamic &startTimeVectorOrig)
            {
                VectorDynamic startTimeVector = RecoverStartTimeVector(
                    startTimeVectorOrig, maskForEliminate, mapIndex);
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                // add dependency constraints
                for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
                {
                    const TaskSet &tasksPrev = itr->second;
                    size_t indexNext = itr->first;
                    for (size_t i = 0; i < tasksPrev.size(); i++)
                    {
                        double err = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0) -
                                     ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0) -
                                     tasksPrev[i].executionTime;
                        res(indexRes++, 0) = Barrier(err);
                    }
                }
                return res;
            };

            MatrixDynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
            {
                VectorDynamic startTimeVector = RecoverStartTimeVector(
                    startTimeVectorOrig, maskForEliminate, mapIndex);

                int m = errorDimension;
                LLint n = length;
                // y -> x
                SM_Dynamic j_yx(m, n);
                // go through m
                LLint index_m = 0;

                // add dependency constraints
                // !! Make sure unordered_map go through edges following the same order as did in evaluateError
                for (auto itr = dagTasks.mapPrev.begin(); itr != dagTasks.mapPrev.end(); itr++)
                {
                    const TaskSet &tasksPrev = itr->second;
                    size_t indexNext = itr->first;
                    for (size_t i = 0; i < tasksPrev.size(); i++)
                    {
                        double err = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0) -
                                     ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0) -
                                     tasksPrev[i].executionTime;
                        // err = Barrier(err);
                        if (err >= deltaOptimizer)
                        {
                            index_m++;
                            // continue;
                        }
                        else if (err <= -deltaOptimizer)
                        {
                            LLint index_N_overall = IndexTran_Instance2Overall(indexNext, 0, sizeOfVariables);
                            LLint index_P_overall = IndexTran_Instance2Overall(tasksPrev[i].id, 0, sizeOfVariables);
                            // j_yx(index_m, index_N_overall) = -1;
                            UpdateSM(-1, index_m, index_N_overall, j_yx);
                            UpdateSM(1, index_m++, index_P_overall, j_yx);

                            // j_yx(index_m++, index_P_overall) = 1;
                        }
                        else if (err > -deltaOptimizer && err < deltaOptimizer)
                        {
                            double x1 = ExtractVariable(startTimeVector, sizeOfVariables, indexNext, 0);
                            double x2 = ExtractVariable(startTimeVector, sizeOfVariables, tasksPrev[i].id, 0);
                            double c = tasksPrev[i].executionTime;
                            double errPlus1 = Barrier(x1 + deltaOptimizer - x2 - c);
                            double errMinus1 = Barrier(x1 - deltaOptimizer - x2 - c);
                            double errPlus2 = Barrier(x1 - deltaOptimizer - x2 - c);
                            double errMinus2 = Barrier(x1 + deltaOptimizer - x2 - c);
                            LLint index_N_overall = IndexTran_Instance2Overall(indexNext, 0, sizeOfVariables);
                            LLint index_P_overall = IndexTran_Instance2Overall(tasksPrev[i].id, 0, sizeOfVariables);
                            // j_yx(index_m, index_N_overall) = (errPlus1 - errMinus1) / 2 / deltaOptimizer;
                            UpdateSM((errPlus1 - errMinus1) / 2 / deltaOptimizer, index_m, index_N_overall, j_yx);
                            // j_yx(index_m++, index_P_overall) = (errPlus2 - errMinus2) / 2 / deltaOptimizer;
                            UpdateSM((errPlus2 - errMinus2) / 2 / deltaOptimizer, index_m++, index_P_overall, j_yx);
                        }
                    }
                }

                // x -> x0
                SM_Dynamic j_map = JacobianElimination(length, lengthCompressed,
                                                       sizeOfVariables, mapIndex, mapIndex_True2Compress);

                return j_yx * j_map;
            }
            Vector evaluateError(const VectorDynamic &startTimeVector,
                                 boost::optional<Matrix &> H = boost::none) const override
            {
                BeginTimer("DAG");
                if (H)
                {
                    if (numericalJaobian)
                        *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                    else
                        *H = JacobianAnalytic(startTimeVector);
                    if (debugMode == 1)
                    {
                        std::cout << "The Jacobian matrix of DAG_ConstraintFactor (including makespac) is " << std::endl
                                  << *H << std::endl;
                    }
                    if (debugMode == 1)
                    {
                        // std::cout << "The input startTimeVector is " << startTimeVector <<std::endl;
                        std::cout << "The error vector of DAG is " << Color::blue << f(startTimeVector) << Color::def << std::endl;
                    }
                }
                EndTimer("DAG");
                return f(startTimeVector);
            }
        };

    }

} // namespace RTSS21IC_NLP