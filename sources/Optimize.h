#include <boost/function.hpp>
#include "RegularTasks.h"

using namespace RegularTaskSystem;
// -------------------------------------------------------- from previous optimization begins
/**
 * barrier function for the optimization
 **/
double Barrier(double x)
{
    if (x > 0)
        // return pow(x, 2);
        return weightLogBarrier * log(x);
    else if (x < 0)
    {
        if (TASK_NUMBER == 0)
        {
            cout << red << "Please set TASK_NUMBER!" << def << endl;
            throw;
        }
        return punishmentInBarrier * pow(10, TASK_NUMBER - 3) * pow(1 - x, 1);
    }
    else // it basically means x=0
        return weightLogBarrier *
               log(x + toleranceBarrier);
}

MatrixDynamic NumericalDerivativeDynamicUpper(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                              VectorDynamic x, double deltaOptimizer, int mOfJacobian)
{
    int n = x.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);
    VectorDynamic currErr = h(x);

    for (int i = 0; i < n; i++)
    {
        VectorDynamic xDelta = x;
        xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
        VectorDynamic resPlus;
        resPlus.resize(mOfJacobian, 1);
        resPlus = h(xDelta);
        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
        }
    }
    return jacobian;
}

// -------------------------------------------------------- from previous optimization ends

inline VectorDynamic GenerateVectorDynamic(LLint N)
{
    VectorDynamic v;
    v.resize(N, 1);
    // v.setZero();
    return v;
}

namespace DAG_SPACE
{

    VectorDynamic GenerateInitialStartTime(TaskSet &tasks, vector<LLint> &sizeOfVariables)
    {
        int N = tasks.size();
        LLint length = 0;

        for (int i = 0; i < N; i++)
        {
            length += sizeOfVariables[i];
        }
        VectorDynamic initial = GenerateVectorDynamic(length);
        LLint index = 0;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < sizeOfVariables[i]; j++)
            {
                initial(index, 0) = tasks[i].period * j;
                index++;
            }
        }
        return initial;
    }

    /**
     * @brief extract instance from the large vector
     * 
     * @param startTimeVector large, combined vector of start time for all instances
     * @param taskIndex task-index
     * @param instanceIndex instance-index
     * @return double start time of the extracted instance
     */
    double ExtractVariable(const VectorDynamic &startTimeVector, const vector<LLint> &sizeOfVariables, int taskIndex, int instanceIndex)
    {
        LLint firstTaskInstance = 0;
        for (int i = 0; i < taskIndex; i++)
        {
            firstTaskInstance += sizeOfVariables[i];
        }
        return startTimeVector(firstTaskInstance + instanceIndex, 0);
    }

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

    // we need to declare all kinds of factors, corresponding to different type of constraints

    // we need a main optimization process, given a DAG input, and return all the start time for all the instances

    // we need an initialization method for phase-1 problem
    class DAG_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint length;

        DAG_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size())
        {
            length = 0;

            for (int i = 0; i < N; i++)
            {
                length += sizeOfVariables[i];
            }
        }

        Vector evaluateError(const VectorDynamic &startTimeVector, boost::optional<Matrix &> H = boost::none) const override
        {

            boost::function<Matrix(const VectorDynamic &)> f =
                [this](const VectorDynamic &startTimeVector)
            {
                // dependency, self DDL, sensor fusion, event chain
                VectorDynamic res = GenerateVectorDynamic(1);
                double rhs = startTimeVector(startTimeVector.rows() - 1, 0);

                // add dependency constraints
                res(0, 0) += Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                     ExtractVariable(startTimeVector, sizeOfVariables, 0, 0) - tasks[0].executionTime + rhs);
                res(0, 0) += Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                     ExtractVariable(startTimeVector, sizeOfVariables, 1, 0) - tasks[1].executionTime + rhs);
                res(0, 0) += Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                     ExtractVariable(startTimeVector, sizeOfVariables, 2, 0) - tasks[2].executionTime + rhs);

                //demand bound function
                for (int i = 0; i < N; i++)
                {
                    double startTime_i = ExtractVariable(startTimeVector, sizeOfVariables, i, 0);
                    for (int j = 0; j < N; j++)
                    {
                        double sumIJK = 0;
                        double startTime_j = ExtractVariable(startTimeVector, sizeOfVariables, j, 0);
                        for (int k = 0; k < N; k++)
                        {
                            if (k == i || k == j)
                                continue;
                            double startTime_k = ExtractVariable(startTimeVector, sizeOfVariables, k, 0);
                            sumIJK += ComputationTime_IJK(startTime_i, tasks[i], startTime_j, tasks[j], startTime_k, tasks[k]);
                            res(0, 0) += Barrier(startTime_j + tasks[j].executionTime - startTime_i - sumIJK + rhs);
                        }
                    }
                }

                // self DDL
                for (int i = 0; i < N; i++)
                {

                    for (int j = 0; j < int(sizeOfVariables[i]); j++)
                    {
                        // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
                        res(0, 0) += Barrier(tasks[i].deadline + i * tasks[i].period -
                                             ExtractVariable(startTimeVector, sizeOfVariables, i, j) - tasks[i].executionTime + rhs);
                        // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                        res(0, 0) += Barrier(ExtractVariable(startTimeVector, sizeOfVariables, i, j) - (i * tasks[i].period) + rhs);
                    }
                }
                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, 1);
            }

            return f(startTimeVector);
        }
    };

    /**
     * @brief Generate initial solution for the whole optimization
     * 
     * @param tasks 
     * @param sizeOfVariables 
     * @return VectorDynamic size (N+1), first N is start time for nodes, the last one is r.h.s.
     */
    VectorDynamic GenerateInitialForDAG(TaskSet &tasks, vector<LLint> &sizeOfVariables)
    {
        int N = tasks.size();
        VectorDynamic initial = GenerateVectorDynamic(N + 1);
        vector<double> executionTimeVec = GetParameter<double>(tasks, "executionTime");
        for (int i = 0; i < N; i++)
        {
            initial(i, 0) = 0;
            initial(N, 0) += executionTimeVec[i];
        }
        return initial;
    }

    /**
     * @brief Given a example regular task sets, perform instance-level optimization
     * 
     * Example DAG system structure:
        s1 -------- Task0 -------- |
                                   |
        s2 -------- Task1 -------- | -------- Task3 -------- Task4
                                   |
        s3 -------- Task2 -------- |
        
        Event chain: s3 - Task3 - Task4
        Sensor chain: s1 - Task0 - Task3 - Task4

        All tasks are non-preemptive;

        All nodes have the same frequency;

        --------------------------------------------------------------------------------------

        Within this optimizataion problem:
        - variables: start time of all the task instances
        - constraints:
            - DAG dependency
            - self DDL constraints, each instance must complete before the next period begins
            - sensor fusion
            - event chain RTA
     * @return all the instances' start time
     */
    VectorDynamic OptimizeTaskSystem1()
    {
        using namespace RegularTaskSystem;
        TaskSet tasks = ReadTaskSet("../TaskData/test_n5_v1.csv", "orig");
        int N = tasks.size();
        LLint hyperPeriod = HyperPeriod(tasks);

        // declare variables
        vector<LLint> sizeOfVariables;
        int variableDimension = 0;
        for (int i = 0; i < N; i++)
        {

            LLint size = hyperPeriod / tasks[i].period;
            sizeOfVariables.push_back(size);
            variableDimension += size;
        }

        // build the factor graph
        auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        NonlinearFactorGraph graph;
        Symbol key('a', 0);
        graph.emplace_shared<DAG_ConstraintFactor>(key, tasks, sizeOfVariables, model);

        VectorDynamic initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables);
        Values initialEstimateFG;
        initialEstimateFG.insert(key, initialEstimate);

        Values result;
        if (optimizerType == 1)
        {
            DoglegParams params;
            if (debugMode == 1)
                params.setVerbosityDL("VERBOSE");
            params.setDeltaInitial(deltaInitialDogleg);
            params.setRelativeErrorTol(relativeErrorTolerance);
            DoglegOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }
        else if (optimizerType == 2)
        {
            LevenbergMarquardtParams params;
            params.setlambdaInitial(initialLambda);
            if (debugMode == 1)
                params.setVerbosityLM("SUMMARY");
            params.setlambdaLowerBound(lowerLambda);
            params.setlambdaUpperBound(upperLambda);
            params.setRelativeErrorTol(relativeErrorTolerance);
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
            result = optimizer.optimize();
        }

        VectorDynamic optComp = result.at<VectorDynamic>(key);
        return optComp;
    }
}