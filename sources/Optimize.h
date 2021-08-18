#include <boost/function.hpp>
#include "RegularTasks.h"

using namespace RegularTaskSystem;
// -------------------------------------------------------- from previous optimization begins
/**
 * barrier function for the optimization
 **/
double Barrier(double x)
{
    if (x >= 0)
        // return pow(x, 2);
        return 0;
    else if (x < 0)
    {
        return -1 * x;
    }
    else // it basically means x=0
        return weightLogBarrier *
               log(x + toleranceBarrier);
}

double BarrierLog(double x)
{
    if (x >= 0)
        // return pow(x, 2);
        return -1 * log(x + 1) + barrierBase;
    else if (x < 0)
    {
        return punishmentInBarrier * pow(1 - x, 2);
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
    // if (debugMode == 1)
    // {
    //     cout << "currErr" << currErr << endl
    //          << endl;
    // }

    for (int i = 0; i < n; i++)
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
        // if (debugMode == 1)
        // {
        //     cout << "resPlus" << resPlus << endl;
        // }

        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
            // jacobian(j, i) = (resMinus(j, 0) - currErr(j, 0)) / deltaOptimizer * -1;
            // jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
        }
    }
    return jacobian;
}

// -------------------------------------------------------- from previous optimization ends

inline VectorDynamic GenerateVectorDynamic(LLint N)
{
    VectorDynamic v;
    v.resize(N, 1);
    v.setZero();
    return v;
}

namespace DAG_SPACE
{

    // VectorDynamic GenerateInitialStartTime(TaskSet &tasks, vector<LLint> &sizeOfVariables)
    // {
    //     int N = tasks.size();
    //     LLint length = 0;

    //     for (int i = 0; i < N; i++)
    //     {
    //         length += sizeOfVariables[i];
    //     }
    //     VectorDynamic initial = GenerateVectorDynamic(length);
    //     // LLint index = 0;
    //     // for (int i = 0; i < N; i++)
    //     // {
    //     //     for (int j = 0; j < sizeOfVariables[i]; j++)
    //     //     {
    //     //         initial(index, 0) = tasks[i].period * j;
    //     //         index++;
    //     //     }
    //     // }
    //     initial << 0, 10, 20, 25, 30, 60;
    //     return initial;
    // }

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
        LLint errorDimension;
        LLint length;

        DAG_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size()), errorDimension(errorDimension)
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
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                // dependency, self DDL, sensor fusion, event chain
                // minimize makespan
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 4, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 0, 0) + 0) *
                                     makespanWeight;

                // add dependency constraints
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 0, 0) - tasks[0].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 1, 0) - tasks[1].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 2, 0) - tasks[2].executionTime + 0);
                res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, 4, 0) -
                                             ExtractVariable(startTimeVector, sizeOfVariables, 3, 0) - tasks[3].executionTime + 0);

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DAG_ConstraintFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector is " << f(startTimeVector) << endl;
                }
            }

            return f(startTimeVector);
        }
    };

    class DBF_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;

        DBF_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size()), errorDimension(errorDimension)
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
                    cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector is " << f(startTimeVector) << endl;
                }
            }

            return f(startTimeVector);
        }
    };

    class DDL_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;

        DDL_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                             SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                       tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                       N(tasks.size()), errorDimension(errorDimension)
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
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                // self DDL
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < int(sizeOfVariables[i]); j++)
                    {
                        // this factor is explained as: variable * 1 < tasks[i].deadline + i * tasks[i].period
                        res(indexRes++, 0) = Barrier(tasks[i].deadline + j * tasks[i].period -
                                                     ExtractVariable(startTimeVector, sizeOfVariables, i, j) - tasks[i].executionTime + 0);
                        // this factor is explained as: variable * -1 < -1 *(i * tasks[i].period)
                        res(indexRes++, 0) = Barrier(ExtractVariable(startTimeVector, sizeOfVariables, i, j) - (j * tasks[i].period) + 0);
                    }
                }

                if (indexRes != errorDimension)
                {
                    cout << red << "The errorDimension is set wrong!" << def << endl;
                    throw;
                }

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of DDL_ConstraintFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector is " << f(startTimeVector) << endl;
                }
            }

            return f(startTimeVector);
        }
    };

    double ExtractMaxDistance(vector<double> &sourceFinishTime)
    {
        return *max_element(sourceFinishTime.begin(), sourceFinishTime.end()) -
               *min_element(sourceFinishTime.begin(), sourceFinishTime.end());
    }
    class SensorFusion_ConstraintFactor : public NoiseModelFactor1<VectorDynamic>
    {
    public:
        TaskSet tasks;
        vector<LLint> sizeOfVariables;
        int N;
        LLint errorDimension;
        LLint length;
        double sensorFusionTol;

        SensorFusion_ConstraintFactor(Key key, TaskSet &tasks, vector<LLint> sizeOfVariables, LLint errorDimension,
                                      double sensorFusionTol,
                                      SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                                tasks(tasks), sizeOfVariables(sizeOfVariables),
                                                                N(tasks.size()), sensorFusionTol(sensorFusionTol),
                                                                errorDimension(errorDimension)
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
                VectorDynamic res;
                res.resize(errorDimension, 1);
                LLint indexRes = 0;

                vector<double> sourceFinishTime;
                // TODO: this is a customized number
                sourceFinishTime.reserve(3);
                // go through all the instances of task 3
                for (int instance_j = 0; instance_j < sizeOfVariables[3]; instance_j++)
                {
                    double startTimeCurr = ExtractVariable(startTimeVector, sizeOfVariables, 3, instance_j);
                    for (int k = 0; k < 3; k++)
                    {
                        LLint instance_l = floor(startTimeCurr / tasks[k].period);
                        double startTime_k_l = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_l);
                        if (startTime_k_l < startTimeCurr)
                            sourceFinishTime.push_back(startTime_k_l + tasks[k].executionTime);
                        else if (instance_l - 1 >= 0)
                        {
                            double startTime_k_l_prev = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_l - 1);
                            if (startTime_k_l_prev < startTimeCurr)
                                sourceFinishTime.push_back(startTime_k_l_prev + tasks[k].executionTime);
                            else
                            {
                                cout << "Error in SensorFusion_ConstraintFactor" << endl;
                                throw;
                            }
                        }
                        // there is no previous instance, i.e., DAG constraints are violated,
                        // find a later instance, and return it
                        else
                        {
                            double startTime_k_l_next = ExtractVariable(startTimeVector, sizeOfVariables, k, instance_l + 1);
                            sourceFinishTime.push_back(startTime_k_l_next + tasks[k].executionTime);
                        }
                    }
                    res(indexRes++, 0) = Barrier(sensorFusionTol - ExtractMaxDistance(sourceFinishTime));
                }

                // res(0, 0) = BarrierLog(sensorFusionTol - ExtractMaxDistance(sourceFinishTime));

                return res;
            };

            if (H)
            {
                *H = NumericalDerivativeDynamicUpper(f, startTimeVector, deltaOptimizer, errorDimension);
                // *H = numericalDerivative11(f, startTimeVector, deltaOptimizer);
                if (debugMode == 1)
                {
                    cout << "The Jacobian matrix of SensorFusion_ConstraintFactor is " << endl
                         << *H << endl;
                }
                if (debugMode == 1)
                {
                    cout << "The input startTimeVector is " << startTimeVector << endl;
                    cout << "The error vector is " << f(startTimeVector) << endl;
                }
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
    VectorDynamic GenerateInitialForDAG(TaskSet &tasks, vector<LLint> &sizeOfVariables, int variableDimension)
    {
        int N = tasks.size();
        VectorDynamic initial = GenerateVectorDynamic(variableDimension);
        vector<double> executionTimeVec = GetParameter<double>(tasks, "executionTime");
        LLint index = 0;
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < sizeOfVariables[i]; j++)
            {
                initial(index++, 0) = j * tasks[i].period + index;
            }
            // initial(i, 0) = 0;
            // initial(N, 0) += executionTimeVec[i];
        }
        // initial(N, 0) *= 1;
        // initial << 0, 1, 0.4, 1.5, 0.9;
        // initial << 0, 0, 0, 0, 0, 0;
        // initial << 6, 2.5, 2, 0.5, 0;
        // initial << 1, 100, 0, 0, 0, 0;
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

        TaskSet tasks = ReadTaskSet("../TaskData/" + testDataSetName + ".csv", "orig");
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
        NonlinearFactorGraph graph;
        Symbol key('a', 0);

        //  + 4 + 2 * variableDimension;
        LLint errorDimensionDAG = 1 + 4;
        auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
        graph.emplace_shared<DAG_ConstraintFactor>(key, tasks, sizeOfVariables, errorDimensionDAG, model);
        LLint errorDimensionDBF = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
        graph.emplace_shared<DBF_ConstraintFactor>(key, tasks, sizeOfVariables, errorDimensionDBF, model);
        LLint errorDimensionDDL = 2 * variableDimension;
        model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
        graph.emplace_shared<DDL_ConstraintFactor>(key, tasks, sizeOfVariables, errorDimensionDDL, model);
        LLint errorDimensionSF = sizeOfVariables[3];
        model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
        graph.emplace_shared<SensorFusion_ConstraintFactor>(key, tasks, sizeOfVariables,
                                                            errorDimensionSF, sensorFusionTolerance, model);

        VectorDynamic initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables, variableDimension);
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

        Values finalEstimate;
        finalEstimate.insert(key, optComp);
        cout << blue << "The error before optimization is " << graph.error(initialEstimateFG) << def << endl;
        cout << blue << "The error after optimization is " << graph.error(finalEstimate) << def << endl;
        return optComp;
    }
}