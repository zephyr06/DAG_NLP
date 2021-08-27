#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "Interval.h"

namespace DAG_SPACE
{
    using namespace RegularTaskSystem;
    /**
     * @brief get c_{ijk} according to ILP paper
     * 
     * @param startTime_i 
     * @param task_i 
     * @param startTime_j 
     * @param task_j 
     * @param startTime_k 
     * @param task_k 
     * @return double 
     */
    inline double ComputationTime_IJK(double startTime_i, const Task &task_i, double startTime_j,
                                      const Task &task_j, double startTime_k, const Task &task_k)
    {
        if (startTime_i <= startTime_k && startTime_k + task_k.executionTime <= startTime_j + task_j.executionTime)
        {
            return task_k.executionTime;
        }
        else
            return 0;
    }
    class DBF_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        vector<bool> maskForEliminate;
        MAP_Index2Data &mapIndex;

        DBF_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables,
                             LLint errorDimension, MAP_Index2Data &mapIndex,
                             vector<bool> &maskForEliminate, SharedNoiseModel model)
            : NoiseModelFactor1<VectorDynamic>(model, key),
              tasks(tasks), sizeOfVariables(sizeOfVariables),
              N(tasks.size()), errorDimension(errorDimension),
              maskForEliminate(maskForEliminate), mapIndex(mapIndex)
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
        }
        static pair<bool, boost::function<VectorDynamic(const VectorDynamic &)>>
        getMappingFunction(int j, VectorDynamic &startTimeVector)
        {
            bool success;
            FuncV2V f;
            return make_pair(false, f);
        }
        /**
         * @brief for error evaluation; this returns a scalar, 
         * which is the merged version of all DBF error
         * 
         */
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic res;
            res.resize(errorDimension, 1);

            VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                                   maskForEliminate, mapIndex);

            if (overlapMode)
            {
                res = DbfInterval(startTimeVector);
                return res;
            }

            LLint indexRes = 0;

            res(indexRes, 0) = 0;
            //demand bound function
            for (int i = 0; i < N; i++)
            {
                for (LLint instance_i = 0; instance_i < sizeOfVariables[i]; instance_i++)
                {
                    double startTime_i = ExtractVariable(startTimeVector, sizeOfVariables, i, instance_i);
                    for (int j = 0; j < N; j++)
                    {
                        for (LLint instance_j = 0; instance_j < sizeOfVariables[j]; instance_j++)
                        {

                            double sumIJK = 0;
                            double startTime_j = ExtractVariable(startTimeVector, sizeOfVariables, j, instance_j);
                            if (startTime_i <= startTime_j &&
                                startTime_i + tasks[i].executionTime <= startTime_j + tasks[j].executionTime)
                            {
                                for (int k = 0; k < N; k++)
                                {
                                    for (LLint instance_k = 0; instance_k < sizeOfVariables[k]; instance_k++)
                                    {
                                        double startTime_k = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_k);
                                        sumIJK += ComputationTime_IJK(startTime_i, tasks[i], startTime_j, tasks[j], startTime_k, tasks[k]);
                                    }
                                }
                                double valueT = Barrier(startTime_j + tasks[j].executionTime - startTime_i - sumIJK + 0);
                                res(indexRes, 0) += valueT;
                            }
                            else
                            {

                                // res(indexRes++, 0) = 0;
                                continue;
                            }
                        }
                    }
                }
            }
            return res;
        };
        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DBF_ConstraintFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    cout << green << "The input startTimeVector of DBF is " << startTimeVector << def << endl;
                    cout << "The error vector of DBF is " << blue << f(startTimeVector) << def << endl;
                }
            }

            return f(startTimeVector);
        }

        // TODO: find a way to avoid eliminate same variable twice
        void addMappingFunction(VectorDynamic &resTemp,
                                MAP_Index2Data &mapIndex, bool &whetherEliminate,
                                vector<bool> &maskForEliminate)
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(resTemp, maskForEliminate, mapIndex);
            VectorDynamic res;
            res.resize(errorDimension, 1);
            LLint indexRes = 0;

            res(indexRes, 0) = 0;
            //demand bound function

            for (int i = 0; i < N; i++)
            {
                for (LLint instance_i = 0; instance_i < sizeOfVariables[i]; instance_i++)
                {
                    double startTime_i = ExtractVariable(startTimeVector, sizeOfVariables, i, instance_i);
                    for (int j = 0; j < N; j++)
                    {
                        for (LLint instance_j = 0; instance_j < sizeOfVariables[j]; instance_j++)
                        {

                            double sumIJK = 0;
                            double startTime_j = ExtractVariable(startTimeVector, sizeOfVariables, j, instance_j);
                            if (startTime_i <= startTime_j &&
                                startTime_i + tasks[i].executionTime <= startTime_j + tasks[j].executionTime)
                            {
                                for (int k = 0; k < N; k++)
                                {
                                    for (LLint instance_k = 0; instance_k < sizeOfVariables[k]; instance_k++)
                                    {
                                        double startTime_k = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_k);
                                        sumIJK += ComputationTime_IJK(startTime_i, tasks[i], startTime_j, tasks[j], startTime_k, tasks[k]);
                                    }
                                }
                                double distanceToBound = startTime_j + tasks[j].executionTime - startTime_i - sumIJK;
                                if (distanceToBound < toleranceEliminator && distanceToBound >= 0)
                                {
                                    LLint index_i_overall = IndexTran_Instance2Overall(i, instance_i, sizeOfVariables);
                                    LLint index_j_overall = IndexTran_Instance2Overall(j, instance_j, sizeOfVariables);
                                    if (index_i_overall == index_j_overall)
                                    {
                                        // this is a self interval, no need to replace one task with itself
                                        continue;
                                    }
                                    if (not maskForEliminate[index_j_overall])
                                    {
                                        maskForEliminate[index_j_overall] = true;
                                        whetherEliminate = true;
                                        MappingDataStruct m{index_i_overall, sumIJK - tasks[j].executionTime};
                                        mapIndex[index_j_overall] = m;
                                    }
                                    else
                                        continue;
                                }
                                else
                                    continue;
                            }
                            else
                            {

                                // res(indexRes++, 0) = 0;
                                continue;
                            }
                        }
                    }
                }
            }
        }

        VectorDynamic DbfInterval(const VectorDynamic &startTimeVector)
        {

            LLint n = startTimeVector.rows();
            vector<Interval> intervalVec;
            intervalVec.reserve(n);
            for (size_t i = 0; i < n; i++)
            {
                double start = startTimeVector(i, 0);
                double length = tasks[BigIndex2TaskIndex(i, sizeOfVariables)].executionTime;
                intervalVec.push_back(Interval{start, length});
            }
            sort(intervalVec.begin(), intervalVec.end(), compare);

            double overlapAll = 0;
            for (size_t i = 0; i < n; i++)
            {
                double endTime = intervalVec[i].start + intervalVec[i].length;
                for (size_t j = i + 1; j < n; j++)
                {
                    if (intervalVec[j].start >= endTime)
                        break;
                    else
                        overlapAll += Overlap(intervalVec[i], intervalVec[j]);
                }
            }

            VectorDynamic res;
            res.resize(1, 1);
            res(0, 0) = overlapAll;
            return res;
        }
    };
}