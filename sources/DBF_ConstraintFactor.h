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
                res = DbfIntervalOverlapError(startTimeVector);
                return res;
            }
            else
            {
                CoutError("You should consider overlapMode, otherwise please comment this and the following lines.");
            }

            // LLint indexRes = 0;

            // res(indexRes, 0) = 0;
            // //demand bound function
            // for (int i = 0; i < N; i++)
            // {
            //     for (LLint instance_i = 0; instance_i < sizeOfVariables[i]; instance_i++)
            //     {
            //         double startTime_i = ExtractVariable(startTimeVector, sizeOfVariables, i, instance_i);
            //         for (int j = 0; j < N; j++)
            //         {
            //             for (LLint instance_j = 0; instance_j < sizeOfVariables[j]; instance_j++)
            //             {

            //                 double sumIJK = 0;
            //                 double startTime_j = ExtractVariable(startTimeVector, sizeOfVariables, j, instance_j);
            //                 if (startTime_i <= startTime_j &&
            //                     startTime_i + tasks[i].executionTime <= startTime_j + tasks[j].executionTime)
            //                 {
            //                     for (int k = 0; k < N; k++)
            //                     {
            //                         for (LLint instance_k = 0; instance_k < sizeOfVariables[k]; instance_k++)
            //                         {
            //                             double startTime_k = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_k);
            //                             sumIJK += ComputationTime_IJK(startTime_i, tasks[i], startTime_j, tasks[j], startTime_k, tasks[k]);
            //                         }
            //                     }
            //                     double valueT = Barrier(startTime_j + tasks[j].executionTime - startTime_i - sumIJK + 0);
            //                     res(indexRes, 0) += valueT;
            //                 }
            //                 else
            //                 {

            //                     // res(indexRes++, 0) = 0;
            //                     continue;
            //                 }
            //             }
            //         }
            //     }
            // }
            return res;
        };
        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {

            if (H)
            {
                *H = NumericalDerivativeDynamicUpperDBF(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DBF_ConstraintFactor is " << endl
                         << *H << endl;
                    cout << Color::green << "The input startTimeVector of DBF is " << startTimeVector << Color::def << endl;
                    cout << "The error vector of DBF is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
                if ((*H).norm() == 0 && f(startTimeVector).norm() != 0)
                {
                    CoutWarning("DBF factor: 0 Jacobian while non-zero error found!");
                }
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DBF_ConstraintFactor is " << endl
                         << *H << endl;
                    cout << Color::green << "The input startTimeVector of DBF is " << startTimeVector << Color::def << endl;
                    cout << "The error vector of DBF is " << Color::blue << f(startTimeVector) << Color::def << endl;
                }
            }

            return f(startTimeVector);
        }

        // /**
        //  * @brief this function performs in-place modification; return results are stored at
        //  * whetherEliminate and mapIndex!
        //  *
        //  * @param index_i_overall
        //  * @param maskForEliminate
        //  * @param whetherEliminate
        //  * @param mapIndex
        //  */
        // void Eliminate_j_based_i(LLint index_j_overall, LLint index_i_overall,
        //                          vector<bool> &maskForEliminate, bool &whetherEliminate,
        //                          MAP_Index2Data &mapIndex, VectorDynamic &startTimeVector,
        //                          double sumIJK, TaskSet &tasks, int j)
        // {
        //     maskForEliminate[index_j_overall] = true;
        //     whetherEliminate = true;
        //     // this should respect original relationship
        //     if (tightEliminate == 1)
        //     {
        //         MappingDataStruct m{index_i_overall, sumIJK - tasks[j].executionTime};
        //         mapIndex[index_j_overall] = m;
        //     }
        //     else if (tightEliminate == 0)
        //     {
        //         MappingDataStruct m{index_i_overall,
        //                             startTimeVector(index_j_overall, 0) -
        //                                 startTimeVector(index_i_overall, 0)};
        //         mapIndex[index_j_overall] = m;
        //     }
        //     else
        //     {
        //         CoutError("Eliminate option error, not recognized!");
        //     }
        // }
        // TODO: find a way to avoid eliminate same variable twice
        void addMappingFunction(VectorDynamic &resTemp,
                                MAP_Index2Data &mapIndex, bool &whetherEliminate,
                                vector<bool> &maskForEliminate,
                                Graph &eliminationTrees,
                                indexVertexMap &indexesBGL)
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
                            if (i == j && instance_i == instance_j)
                            {
                                // this is a self interval, no need to replace one task with itself
                                continue;
                            }

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
                                // computation speed can be improved by 1 times there
                                double distanceToBound_j_i = startTime_j + tasks[j].executionTime - startTime_i - sumIJK;
                                if (distanceToBound_j_i < toleranceEliminator && distanceToBound_j_i >= 0)
                                {
                                    LLint index_i_overall = IndexTran_Instance2Overall(i, instance_i, sizeOfVariables);
                                    LLint index_j_overall = IndexTran_Instance2Overall(j, instance_j, sizeOfVariables);

                                    // since we go over all the pairs, we only need to check j in each pair (i, j)
                                    if (not maskForEliminate[index_j_overall])
                                    // this if condition is posed to avoid repeated elimination, and avoid conflicting elimination
                                    // because one variable j can only depend on one single variable i;
                                    {
                                        // Eliminate_j_based_i(index_j_overall, index_i_overall,
                                        //                     maskForEliminate, whetherEliminate,
                                        //                     mapIndex, startTimeVector, sumIJK, tasks, j);

                                        // check eliminationTrees confliction; only preceed if no confliction exists
                                        vector<LLint> tree_i, tree_j;
                                        Vertex u1 = indexesBGL[index_i_overall];
                                        Vertex v1 = indexesBGL[index_j_overall];
                                        FindSubTree(eliminationTrees, tree_i, u1);
                                        FindSubTree(eliminationTrees, tree_j, v1);
                                        if (CheckNoConflictionTree(tree_i, tree_j, startTimeVector))
                                        // if (true)
                                        {
                                            maskForEliminate[index_j_overall] = true;
                                            whetherEliminate = true;
                                            // this should respect original relationship
                                            if (tightEliminate == 1)
                                            {
                                                MappingDataStruct m{index_i_overall, sumIJK - tasks[j].executionTime};
                                                mapIndex[index_j_overall] = m;
                                            }
                                            else if (tightEliminate == 0)
                                            {
                                                double distt = startTimeVector(index_j_overall, 0) -
                                                               startTimeVector(index_i_overall, 0);
                                                if (distt >= 142.0 && distt <= 143.0)
                                                    int a = 1;
                                                if (distt >= 54 && distt <= 55.0)
                                                    int a = 1;
                                                if (distt >= 28.0 && distt <= 29.0)
                                                    int a = 1;
                                                MappingDataStruct m{index_i_overall,
                                                                    distt};
                                                mapIndex[index_j_overall] = m;
                                            }
                                            else
                                            {
                                                CoutError("Eliminate option error, not recognized!");
                                            }
                                            // add edge to eliminationTrees
                                            graph_traits<Graph>::edge_descriptor e;
                                            bool inserted;
                                            boost::tie(e, inserted) = add_edge(indexesBGL[index_j_overall],
                                                                               indexesBGL[index_i_overall],
                                                                               eliminationTrees);
                                            if (inserted)
                                            {
                                                edge_name_map_t edgeMapCurr = get(edge_name, eliminationTrees);
                                                edgeMapCurr[e] = mapIndex[index_j_overall].getDistance();
                                            }
                                        }
                                        else
                                        {
                                            continue;
                                        }
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
        vector<Interval> CreateIntervalFromSTV(vector<LLint> &tree1, const VectorDynamic &startTimeVector)
        {
            size_t n = tree1.size();
            vector<Interval> intervalVec;
            intervalVec.reserve(n);
            for (size_t i = 0; i < n; i++)
            {
                size_t index = tree1[i];
                double start = startTimeVector(index, 0);
                double length = tasks[BigIndex2TaskIndex(index, sizeOfVariables)].executionTime;
                intervalVec.push_back(Interval{start, length});
            }
            return intervalVec;
        }

        VectorDynamic DbfIntervalOverlapError(const VectorDynamic &startTimeVector)
        {
            vector<LLint> indexes;
            indexes.reserve(startTimeVector.size());
            for (size_t i = 0; i < startTimeVector.size(); i++)
                indexes.push_back(i);
            vector<Interval> intervalVec = CreateIntervalFromSTV(indexes, startTimeVector);

            VectorDynamic res;
            res.resize(1, 1);
            res(0, 0) = IntervalOverlapError(intervalVec);
            return res;
        }

        bool CheckNoConflictionTree(const vector<LLint> &tree1, const vector<LLint> &tree2,
                                    const VectorDynamic &startTimeVector)
        {
            // TODO: separate this checking process can improve speed
            vector<LLint> trees;
            LLint ssize = tree1.size() + tree2.size();
            trees.reserve(ssize);
            for (size_t i = 0; i < tree1.size(); i++)
                trees.push_back(tree1[i]);
            for (size_t i = 0; i < tree2.size(); i++)
                trees.push_back(tree2[i]);

            vector<Interval> vv = CreateIntervalFromSTV(trees, startTimeVector);
            double error_I_O = IntervalOverlapError(vv);
            if (error_I_O == 0)
                return true;
            else if (error_I_O > 0)
                return false;
            else
            {
                CoutError("IntervalOverlapError returns negative!");
            }
            return false;
        }
    };
}