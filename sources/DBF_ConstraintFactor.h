#include "unordered_map"
#include "DeclareDAG.h"
#include "RegularTasks.h"
#include "DBF_utils.h"
// coreNumberAva = 1;
namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    class DBF_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        vector<bool> maskForEliminate;
        MAP_Index2Data mapIndex;
        LLint lengthCompressed;
        // each element contains tasks belonging to the same processor

        ProcessorTaskSet processorTasks;
        std::unordered_map<LLint, LLint> mapIndex_True2Compress;

        DBF_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables,
                             LLint errorDimension, MAP_Index2Data &mapIndex,
                             vector<bool> &maskForEliminate, ProcessorTaskSet &processorTasks,
                             SharedNoiseModel model)
            : NoiseModelFactor1<VectorDynamic>(model, key),
              tasks(tasks), sizeOfVariables(sizeOfVariables),
              N(tasks.size()), errorDimension(errorDimension),
              maskForEliminate(maskForEliminate), mapIndex(mapIndex),
              processorTasks(processorTasks)
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
        /**
         * @brief for error evaluation; this returns a VectorDynamic, 
         * which is the merged version of all DBF error for each processor
         * 
         */
        boost::function<Matrix(const VectorDynamic &)> f =
            [this](const VectorDynamic &startTimeVectorOrig)
        {
            VectorDynamic res;
            res.resize(errorDimension, 1);

            VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                                   maskForEliminate, mapIndex);

            int indexPro = 0;
            for (auto itr = processorTasks.begin(); itr != processorTasks.end(); itr++)
            {
                res(indexPro++, 0) = DbfIntervalOverlapError(startTimeVector, itr->first,
                                                             processorTasks, tasks, sizeOfVariables);
            }

            return res;
        };
        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {
            BeginTimer("DBF_All");
            if (H)
            {
                // BeginTimer("DBF_H");
                if (numericalJaobian)
                {
                    *H = NumericalDerivativeDynamicUpperDBF(f, startTimeVector, deltaOptimizer, errorDimension);
                }
                else
                    *H = DBFJacobian(f, startTimeVector, deltaOptimizer, errorDimension);
                // EndTimer("DBF_H");
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DBF_ConstraintFactor is " << endl
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
                    cout << "The Jacobian matrix of DBF_ConstraintFactor is " << endl
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
         * @brief detecting elimination and update elimination records for both mapIndex and eliminationTrees_Update
         * 
         * @param resTemp 
         * @param mapIndex 
         * @param whetherEliminate 
         * @param maskForEliminate_addMap 
         * @param eliminationTrees_Update 
         * @param indexesBGL_Update properties access for eliminationTrees_Update
         */
        void addMappingFunction(VectorDynamic &resTemp,
                                MAP_Index2Data &mapIndex, bool &whetherEliminate,
                                vector<bool> &maskForEliminate_addMap,
                                Graph &eliminationTrees_Update,
                                indexVertexMap &indexesBGL_Update)
        {
            BeginTimer("addMap2");

            VectorDynamic startTimeVector = RecoverStartTimeVector(resTemp, maskForEliminate_addMap, mapIndex);

            for (auto itr = processorTasks.begin(); itr != processorTasks.end(); itr++)
            {
                int processorCurr = itr->first;
                vector<int> tasksCurr = itr->second;
                vector<Interval> intervalVec = DbfInterval(startTimeVector, processorCurr,
                                                           processorTasks, tasks, sizeOfVariables);
                sort(intervalVec.begin(), intervalVec.end(), compare);

                // find DBF error that need elimination
                for (LLint i = 0; i < LLint(intervalVec.size()); i++)
                {
                    for (LLint j = i + 1; j < LLint(intervalVec.size()); j++)
                    {
                        // this if condition is posed to avoid repeated elimination, and avoid conflicting elimination
                        // because one variable j can only depend on one single variable i;
                        if (maskForEliminate_addMap[intervalVec[j].indexInSTV])
                        {
                            continue;
                        }
                        double sumIJK = 0;
                        double endTime = intervalVec[j].start + intervalVec[j].length;
                        for (LLint k = 0; k < LLint(intervalVec.size()); k++)
                        {
                            if (intervalVec[k].start >= endTime)
                                break;
                            sumIJK += ComputationTime_IJK(intervalVec[i],
                                                          intervalVec[j],
                                                          intervalVec[k]);
                        }
                        double distanceToBound_j_i = intervalVec[j].start + intervalVec[j].length - intervalVec[i].start - sumIJK;

                        if ((distanceToBound_j_i < toleranceEliminator && distanceToBound_j_i >= 0))
                        {
                            LLint index_i_overall = intervalVec[i].indexInSTV;
                            LLint index_j_overall = intervalVec[j].indexInSTV;

                            // since we go over all the pairs, we only need to check j in each pair (i, j)

                            vector<LLint> tree_i, tree_j;
                            Vertex u1 = indexesBGL_Update[index_i_overall];
                            Vertex v1 = indexesBGL_Update[index_j_overall];
                            FindSubTree(eliminationTrees_Update, tree_i, u1);
                            FindSubTree(eliminationTrees_Update, tree_j, v1);
                            if (CheckNoConflictionTree(tree_i, tree_j, startTimeVector))
                            {
                                maskForEliminate_addMap[index_j_overall] = true;
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

                                    MappingDataStruct m{index_i_overall,
                                                        distt};
                                    mapIndex[index_j_overall] = m;
                                }
                                else
                                {
                                    CoutError("Eliminate option error, not recognized!");
                                }
                                // add edge to eliminationTrees_Update
                                graph_traits<Graph>::edge_descriptor e;
                                bool inserted;
                                boost::tie(e, inserted) = add_edge(indexesBGL_Update[index_j_overall],
                                                                   indexesBGL_Update[index_i_overall],
                                                                   eliminationTrees_Update);
                                // if (inserted)
                                // {
                                //     edge_name_map_t edgeMapCurr = get(edge_name, eliminationTrees_Update);
                                //     edgeMapCurr[e] = mapIndex[index_j_overall].getDistance();
                                // }
                            }
                            else
                            {
                                continue;
                            }
                        }
                        else
                            continue;
                    }
                }
            }
            EndTimer("addMap2");
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

            vector<Interval> vv = CreateIntervalFromSTVSameOrder(trees, startTimeVector, tasks, sizeOfVariables);
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

        // Following parts are used for numerical Jacobian **********************************************

        /**
         * @brief FindVanishIndex; given a startTimeVector, some of their intervals may be fully overlapped
         * by another, and this function finds all the indexes that is related.
         * 
         * @param startTimeVectorOrig 
         * @return vector<LLint> 
         */
        vector<LLint> FindVanishIndex(const VectorDynamic &startTimeVectorOrig) const
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(startTimeVectorOrig,
                                                                   maskForEliminate, mapIndex);
            vector<LLint> indexes;
            indexes.reserve(startTimeVector.size());
            for (uint i = 0; i < startTimeVector.size(); i++)
                indexes.push_back(i);
            vector<Interval> intervalVec = CreateIntervalFromSTVSameOrder(indexes, startTimeVector, tasks, sizeOfVariables);

            LLint variableDimension = intervalVec.size();
            vector<LLint> coverIntervalIndex;
            coverIntervalIndex.reserve(variableDimension);
            std::unordered_set<LLint> indexSetBig;

            for (LLint i = 0; i < variableDimension; i++)
            {
                for (LLint j = i + 1; j < variableDimension; j++)
                {
                    double s1 = intervalVec[i].start;
                    double f1 = s1 + intervalVec[i].length;
                    double s2 = intervalVec[j].start;
                    double f2 = s2 + intervalVec[j].length;
                    if ((s2 > s1 && f2 < f1) || (s2 < s1 && f2 > f1))
                    {
                        LLint leafIndex = FindLeaf(i, mapIndex);
                        if (indexSetBig.find(leafIndex) == indexSetBig.end())
                        {
                            indexSetBig.insert(leafIndex);
                        }
                        leafIndex = FindLeaf(j, mapIndex);
                        if (indexSetBig.find(leafIndex) == indexSetBig.end())
                        {
                            indexSetBig.insert(leafIndex);
                        }
                    }
                }
            }

            auto m = MapIndex_True2Compress(maskForEliminate);
            vector<LLint> coverIndexInCompressed;
            coverIndexInCompressed.reserve(startTimeVectorOrig.rows());
            for (auto itr = indexSetBig.begin(); itr != indexSetBig.end(); itr++)
            {
                coverIndexInCompressed.push_back(m[(*itr)]);
            }
            return coverIndexInCompressed;
        }

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
            int n = x.rows();
            MatrixDynamic jacobian;
            jacobian.resize(mOfJacobian, n);
            jacobian.setZero();

            vector<LLint> vanishGradientIndex = FindVanishIndex(x);
            std::unordered_set<LLint> ss;
            for (size_t i = 0; i < vanishGradientIndex.size(); i++)
            {
                ss.insert(vanishGradientIndex[i]);
            }
            for (int i = 0; i < n; i++)
            {
                // check whether this variable is directly
                if (ss.find(i) == ss.end())
                {
                    VectorDynamic xDelta = x;
                    xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
                    VectorDynamic resPlus;
                    resPlus.resize(mOfJacobian, 1);
                    resPlus = h(xDelta);
                    xDelta(i, 0) = xDelta(i, 0) - 2 * deltaOptimizer;
                    VectorDynamic resMinus;
                    resMinus.resize(mOfJacobian, 1);
                    resMinus = h(xDelta);

                    for (int j = 0; j < mOfJacobian; j++)
                    {
                        jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
                    }
                }
                else
                {
                    int iteration = 0;
                    double deltaInIteration = deltaOptimizer;
                    while (iteration < maxJacobianIteration)
                    {

                        VectorDynamic xDelta = x;
                        xDelta(i, 0) = xDelta(i, 0) + deltaInIteration;
                        VectorDynamic resPlus;
                        resPlus.resize(mOfJacobian, 1);
                        resPlus = h(xDelta);
                        xDelta(i, 0) = xDelta(i, 0) - 2 * deltaInIteration;
                        VectorDynamic resMinus;
                        resMinus.resize(mOfJacobian, 1);
                        resMinus = h(xDelta);
                        if (resPlus == resMinus && resPlus.norm() != 0)
                        {
                            deltaInIteration = deltaInIteration * stepJacobianIteration;
                            iteration++;
                        }
                        else
                        {
                            for (int j = 0; j < mOfJacobian; j++)
                            {
                                jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaInIteration;
                            }
                            break;
                        }
                    }
                }
            }

            return jacobian;
        }

        /**
         * @brief pure analytic Jacobian estimation
         * 
         * @param startTimeVectorOrig 
         * @return MatrixDynamic 
         */
        MatrixDynamic JacobianAnalytic(const VectorDynamic &startTimeVectorOrig) const
        {
            VectorDynamic startTimeVector = RecoverStartTimeVector(
                startTimeVectorOrig, maskForEliminate, mapIndex);

            int m = errorDimension;
            LLint n = length;
            // y -> x
            MatrixDynamic j_yx = GenerateMatrixDynamic(m, n);
            j_yx.resize(m, n);

            int processorIndex = 0;
            for (auto proPtr = processorTasks.begin(); proPtr != processorTasks.end(); proPtr++)
            {
                vector<Interval> intervalVec = DbfInterval(startTimeVector, proPtr->first,
                                                           processorTasks, tasks, sizeOfVariables);
                sort(intervalVec.begin(), intervalVec.end(), compare);

                for (LLint i = 0; i < LLint(intervalVec.size()); i++)
                {
                    double endTime = intervalVec[i].start + intervalVec[i].length;
                    for (LLint j = i + 1; j < LLint(intervalVec.size()); j++)
                    {
                        if (intervalVec[j].start > endTime + deltaOptimizer)
                            break;
                        else
                        {
                            auto gPair = OverlapGradient(intervalVec[i], intervalVec[j]);
                            j_yx(processorIndex, intervalVec[i].indexInSTV) += gPair.first;
                            j_yx(processorIndex, intervalVec[j].indexInSTV) += gPair.second;
                        }
                    }
                }
                processorIndex++;
            }
            SM_Dynamic j_map = JacobianElimination(length, lengthCompressed,
                                                   sizeOfVariables, mapIndex, mapIndex_True2Compress);
            return j_yx * j_map;
        }

        /**
         * @brief estimate analytic Jacobian. To avoid gradient vanish issue, 
         * it also changes step size if necessary
         * 
         * @param h 
         * @param x 
         * @param deltaOptimizer 
         * @param mOfJacobian 
         * @return MatrixDynamic 
         */
        MatrixDynamic DBFJacobian(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                  const VectorDynamic &x, double deltaOptimizer,
                                  int mOfJacobian) const
        {
            int n = x.rows();
            MatrixDynamic jacobian = JacobianAnalytic(x);

            vector<LLint> vanishGradientIndex = FindVanishIndex(x);
            std::unordered_set<LLint> ss;
            for (size_t i = 0; i < vanishGradientIndex.size(); i++)
            {
                ss.insert(vanishGradientIndex[i]);
            }

            for (int i = 0; i < n; i++)
            {
                // check whether this variable is directly
                if (ss.find(i) == ss.end())
                {
                    ;
                }
                else
                {
                    BeginTimer("DBF_H_adjust");
                    int iteration = 0;
                    double deltaInIteration = deltaOptimizer;
                    while (iteration < maxJacobianIteration)
                    {

                        VectorDynamic xDelta = x;
                        xDelta(i, 0) = xDelta(i, 0) + deltaInIteration;
                        VectorDynamic resPlus;
                        resPlus.resize(mOfJacobian, 1);
                        resPlus = h(xDelta);
                        xDelta(i, 0) = xDelta(i, 0) - 2 * deltaInIteration;
                        VectorDynamic resMinus;
                        resMinus.resize(mOfJacobian, 1);
                        resMinus = h(xDelta);
                        if (resPlus == resMinus && resPlus.norm() != 0)
                        {
                            deltaInIteration = deltaInIteration * stepJacobianIteration;
                            iteration++;
                        }
                        else
                        {
                            for (int j = 0; j < mOfJacobian; j++)
                            {
                                jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaInIteration;
                            }
                            break;
                        }
                    }
                    EndTimer("DBF_H_adjust");
                }
            }

            return jacobian;
        }
    };
}