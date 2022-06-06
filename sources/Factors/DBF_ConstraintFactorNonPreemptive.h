#include "unordered_map"
#include "sources/Factors/BaseSchedulingFactor.h"
namespace DAG_SPACE
{
    using namespace RegularTaskSystem;

    void AddDBF_Factor(NonlinearFactorGraph &graph, TaskSetInfoDerived &tasksInfo)
    {

        LLint errorDimensionDBF = 1;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);

        for (int i = 0; i < tasksInfo.N; i++)
        {
            for (int j = 0; j < int(tasksInfo.sizeOfVariables[i]); j++)
            {
                LLint index_overall = IndexTran_Instance2Overall(i, j, tasksInfo.sizeOfVariables);
                Symbol key = GenerateKey(i, j);
                double start = tasksInfo.tasks[i].period * j;
                double end = tasksInfo.tasks[i].period * (j + 1);

                for (int ii = i + 1; ii < tasksInfo.N; ii++)
                {
                    for (int jj = 0; jj < int(tasksInfo.sizeOfVariables[ii]); jj++)
                    {
                        double startInner = tasksInfo.tasks[ii].period * jj;
                        double endInner = tasksInfo.tasks[ii].period * (jj + 1);
                        if (startInner > end || start > endInner)
                        {
                            continue;
                        }

                        LLint index_overall_inner = IndexTran_Instance2Overall(ii, jj, tasksInfo.sizeOfVariables);
                        Symbol key_inner = GenerateKey(ii, jj);
                        Interval v1 = CreateSingleInterval(index_overall, 0.0,
                                                           tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        Interval v2 = CreateSingleInterval(index_overall_inner, 0.0,
                                                           tasksInfo.tasks, tasksInfo.sizeOfVariables);
                        NormalErrorFunction2D DBF2D =
                            [v1, v2](VectorDynamic x1, VectorDynamic x2)
                        {
                            BeginTimer("DBF_Lambda");
                            VectorDynamic res = x1;
                            Interval v11 = v1;
                            Interval v22 = v2;
                            v11.start = x1(0, 0);
                            v22.start = x2(0, 0);

                            res << Overlap(v11, v22);
                            EndTimer("DBF_Lambda");
                            return res;
                        };
                        // this factor is explained as: variable * 1 <= tasks[i].deadline + i * tasks[i].period
                        graph.emplace_shared<InequalityFactor2D>(key, key_inner, DBF2D, model);
                    }
                }
            }
        }
    }
    // *************************************below is not useful*******************************************
    class DBF_ConstraintFactor : public BaseSchedulingFactor
    {
    public:
        DBF_ConstraintFactor(Key key, TaskSetInfoDerived &tasksInfo,
                             EliminationForest &forestInfo, LLint errorDimension,
                             SharedNoiseModel model) : BaseSchedulingFactor(key,
                                                                            tasksInfo,
                                                                            forestInfo,
                                                                            errorDimension,
                                                                            model)
        {
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
                                                                   forestInfo);

            int indexPro = 0;
            for (auto itr = tasksInfo.processorTaskSet.begin(); itr != tasksInfo.processorTaskSet.end(); itr++)
            {
                res(indexPro++, 0) = DbfIntervalOverlapError(startTimeVector, itr->first,
                                                             tasksInfo.processorTaskSet, tasksInfo.tasks, tasksInfo.sizeOfVariables);
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
        // void addMappingFunction(VectorDynamic &resTemp,
        //                         MAP_Index2Data &mapIndex, bool &whetherEliminate,
        //                         vector<bool> &maskForEliminate_addMap,
        //                         Graph &eliminationTrees_Update,
        //                         indexVertexMap &indexesBGL_Update)
        void addMappingFunction(VectorDynamic &resTemp, bool &whetherEliminate,
                                EliminationForest &forestInfo)
        {
            BeginTimer("addMap2");

            VectorDynamic startTimeVector = RecoverStartTimeVector(resTemp, forestInfo);

            for (auto itr = tasksInfo.processorTaskSet.begin(); itr != tasksInfo.processorTaskSet.end(); itr++)
            {
                int processorCurr = itr->first;
                vector<int> tasksCurr = itr->second;
                vector<Interval> intervalVec = DbfInterval(startTimeVector, processorCurr,
                                                           tasksInfo.processorTaskSet, tasksInfo.tasks, tasksInfo.sizeOfVariables);
                sort(intervalVec.begin(), intervalVec.end(), compare);

                // find DBF error that need elimination
                for (LLint i = 0; i < LLint(intervalVec.size()); i++)
                {
                    for (LLint j = i + 1; j < LLint(intervalVec.size()); j++)
                    {
                        // this if condition is posed to avoid repeated elimination, and avoid conflicting elimination
                        // because one variable j can only depend on one single variable i;
                        if (forestInfo.maskForEliminate[intervalVec[j].indexInSTV])
                        {
                            continue;
                        }
                        LLint index_i_overall = intervalVec[i].indexInSTV;
                        LLint index_j_overall = intervalVec[j].indexInSTV;
                        if (FindLeaf(index_i_overall, forestInfo.mapIndex) == FindLeaf(index_j_overall, forestInfo.mapIndex))
                            continue;
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

                            // since we go over all the pairs, we only need to check j in each pair (i, j)

                            vector<LLint> tree_i, tree_j;
                            Vertex u1 = forestInfo.indexesBGL[index_i_overall];
                            Vertex v1 = forestInfo.indexesBGL[index_j_overall];
                            FindSubTree(forestInfo.eliminationTrees, tree_i, u1);
                            FindSubTree(forestInfo.eliminationTrees, tree_j, v1);
                            if (CheckNoConflictionTree(tree_i, tree_j, startTimeVector))
                            {
                                // forestInfo.maskForEliminate[index_j_overall] = true;

                                // // this should respect original relationship
                                // if (tightEliminate == 1)
                                // {
                                //     MappingDataStruct m{index_i_overall, sumIJK - tasks[j].executionTime};
                                //     forestInfo.mapIndex[index_j_overall] = m;
                                // }
                                // else if (tightEliminate == 0)
                                // {
                                // double distt = startTimeVector(index_j_overall, 0) -
                                //                startTimeVector(index_i_overall, 0);

                                //     MappingDataStruct m{index_i_overall,
                                //                         distt};
                                //     forestInfo.mapIndex[index_j_overall] = m;
                                // }
                                // else
                                // {
                                //     CoutError("Eliminate option error, not recognized!");
                                // }
                                // // add edge to eliminationTrees
                                // graph_traits<Graph>::edge_descriptor e;
                                // bool inserted;
                                // boost::tie(e, inserted) = add_edge(forestInfo.indexesBGL[index_j_overall],
                                //                                    forestInfo.indexesBGL[index_i_overall],
                                //                                    forestInfo.eliminationTrees);
                                whetherEliminate = true;
                                if (tightEliminate == 1)
                                {
                                    forestInfo.AddLinearEliminate(index_j_overall, index_i_overall, sumIJK - tasksInfo.tasks[j].executionTime);
                                }
                                else if (tightEliminate == 0)
                                {
                                    double distt = startTimeVector(index_j_overall, 0) -
                                                   startTimeVector(index_i_overall, 0);
                                    forestInfo.AddLinearEliminate(index_j_overall, index_i_overall, distt);
                                }
                                else
                                {
                                    CoutError("Eliminate option error, not recognized!");
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
                }
            }
            EndTimer("addMap2");
        }

        /**
         * @brief true means no conflict, false means exist conflictions
         *
         * @param tree1
         * @param tree2
         * @param startTimeVector
         * @return true
         * @return false
         */
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

            vector<Interval> vv = CreateIntervalFromSTVSameOrder(trees, startTimeVector, tasksInfo.tasks, tasksInfo.sizeOfVariables);
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

            vector<LLint> vanishGradientIndex = FindVanishIndex(x, tasksInfo.tasks, tasksInfo.sizeOfVariables, forestInfo);
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
                    while (iteration < MaxEliminateDetectIterations)
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
                startTimeVectorOrig, forestInfo);

            int m = errorDimension;
            LLint n = tasksInfo.length;
            // y -> x
            MatrixDynamic j_yx = GenerateMatrixDynamic(m, n);
            j_yx.resize(m, n);

            int processorIndex = 0;
            for (auto proPtr = tasksInfo.processorTaskSet.begin(); proPtr != tasksInfo.processorTaskSet.end(); proPtr++)
            {
                vector<Interval> intervalVec = DbfInterval(startTimeVector, proPtr->first,
                                                           tasksInfo.processorTaskSet, tasksInfo.tasks, tasksInfo.sizeOfVariables);
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
            SM_Dynamic j_map = JacobianElimination(tasksInfo, forestInfo);
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

            vector<LLint> vanishGradientIndex = FindVanishIndex(x, tasksInfo.tasks, tasksInfo.sizeOfVariables, forestInfo);
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
                    while (iteration < MaxEliminateDetectIterations)
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