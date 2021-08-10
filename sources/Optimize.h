#include "DeclareDAG.h"
#include "RegularTasks.h"

inline VectorDynamic CreateVectorDynamic(LLint N)
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
        VectorDynamic initial = CreateVectorDynamic(length);
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
    double ExtractVariable(const VectorDynamic &startTimeVector, vector<LLint> &sizeOfVariables, int taskIndex, int instanceIndex)
    {
        LLint firstTaskInstance = 0;
        for (int i = 0; i < taskIndex; i++)
        {
            firstTaskInstance += sizeOfVariables[i];
        }
        return startTimeVector(firstTaskInstance + j, 0);
    }

    

    inline double ComputationTime_IJK(double startTime_i, Task& task_i, double startTime_j, Task& task_j, double startTime_k, Task& task_k)
    {
        if(startTime_i <= startTime_k && startTime_k+task_k.executionTime <= startTime_j+task_j.executionTime)
        {
            return task_k.executionTime;
        }
        else return 0;
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
            // dependency, self DDL, sensor fusion, event chain
            VectorDynamic res = CreateVectorDynamic(N + length * 2 + sizeOfVariables[3] +);

            for (int i = 0; i < N; i++)
            {
                // add dependency constraints
                if (i < N - 1)
                {
                    graph.emplace_shared<DependencyFactor>(keysOfVariables[i][0], keysOfVariables[i + 1][0], model);
                }

                for (int j = 0; j < int(keysOfVariables[i].size()); j++)
                {
                    // self DDL
                    // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
                    graph.emplace_shared<LinearInequalityFactor1D>(keysOfVariables[i][j], 1, tasks[i].deadline + i * tasks[i].period, model);
                    // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                    graph.emplace_shared<LinearInequalityFactor1D>(keysOfVariables[i][j], -1, -1 * (i * tasks[i].period), model);

                    //demand bound function

                }
            }
            return res;
        }
    };

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
        long long int hyperPeriod = HyperPeriod(tasks);

        // declare variables
        vector<long long int> sizeOfVariables;
        int variableDimension = 0;
        for (int i = 0; i < N; i++)
        {

            long long int size = hyperPeriod / tasks[i].period;
            sizeOfVariables.push_back(size);
            variableDimension += size;
        }

        // build the factor graph
        auto model = noiseModel::Isotropic::Sigma(variableDimension, noiseModelSigma);
        NonlinearFactorGraph graph;
        Symbol key('a', 0);
        graph.emplace_shared<DAG_ConstraintsFactor>(key, tasks, sizeOfVariables, model);

        initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables);
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