#include "sources/Optimization/Optimize.h"
#include "sources/Tools/testMy.h"
#include "sources/Optimization/EliminationForest_utils.h"

// ************************************************************

class test1Factor : public NoiseModelFactor1<VectorDynamic>
{
public:
    double c;

    test1Factor(Key key, double c, SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key), c(c) {}
    Vector evaluateError(const VectorDynamic &startTimeVector,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic err = GenerateVectorDynamic(1);
        err(0, 0) = pow(c - startTimeVector(0, 0), 2);
        if (H)
        {
            MatrixDynamic hh = GenerateMatrixDynamic(1, 1);
            hh(0, 0) = 2 * (c - startTimeVector(0, 0)) * -1;
            *H = hh;
        }
        return err;
    }
};
TEST(compareSparsity, v1)
{
    int N = 10;
    BeginTimer("test1");
    VectorDynamic expectedB = GenerateVectorDynamic(N);
    for (int i = 0; i < N; i++)
        expectedB(i, 0) = sin(i);

    NonlinearFactorGraph graph;

    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    Values initialEstimateFG;
    for (int i = 0; i < N; i++)
    {
        Symbol key('a', i);
        graph.emplace_shared<test1Factor>(key, expectedB(i, 0), model);
        VectorDynamic initialEstimate = GenerateVectorDynamic(1);
        initialEstimateFG.insert(key, initialEstimate);
    }

    Values result;
    if (optimizerType == 1)
    {
        DoglegParams params;
        if (debugMode >= 1)
            params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setMaxIterations(maxIterations);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        if (debugMode >= 1)
            params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setMaxIterations(maxIterations);
        params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    Symbol key('a', 0);
    VectorDynamic optComp = result.at<VectorDynamic>(key);
    cout << "Test dummy: " << optComp << endl;
    EndTimer("test1");
}
class test2Factor : public NoiseModelFactor1<VectorDynamic>
{
public:
    VectorDynamic c;
    LLint length;

    test2Factor(Key key, VectorDynamic &c, SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key), c(c)
    {
        length = c.rows();
    }
    Vector evaluateError(const VectorDynamic &startTimeVector,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        VectorDynamic err = GenerateVectorDynamic(length);
        for (int i = 0; i < length; i++)
            err(i, 0) = pow(c(i, 0) - startTimeVector(i, 0), 2);
        if (H)
        {
            SM_Dynamic hh(length, length);
            hh.resize(length, length);
            for (int i = 0; i < length; i++)
                hh.insert(i, i) = 2 * (c(i, 0) - startTimeVector(i, 0)) * -1;
            *H = hh;
        }
        return err;
    }
};
TEST(compareSparsity, v2)
{
    int N = 10;
    BeginTimer("test2");
    VectorDynamic expectedB = GenerateVectorDynamic(N);
    for (int i = 0; i < N; i++)
        expectedB(i, 0) = sin(i);

    NonlinearFactorGraph graph;

    LLint errorDimensionMS = N;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    Values initialEstimateFG;

    Symbol key('a', 0);
    graph.emplace_shared<test2Factor>(key, expectedB, model);
    VectorDynamic initialEstimate = GenerateVectorDynamic(N);
    initialEstimateFG.insert(key, initialEstimate);

    Values result;
    if (optimizerType == 1)
    {
        DoglegParams params;
        if (debugMode >= 1)
            params.setVerbosityDL("VERBOSE");
        params.setDeltaInitial(deltaInitialDogleg);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setMaxIterations(maxIterations);
        DoglegOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    else if (optimizerType == 2)
    {
        LevenbergMarquardtParams params;
        params.setlambdaInitial(initialLambda);
        if (debugMode >= 1)
            params.setVerbosityLM("SUMMARY");
        params.setlambdaLowerBound(lowerLambda);
        params.setlambdaUpperBound(upperLambda);
        params.setRelativeErrorTol(relativeErrorTolerance);
        params.setMaxIterations(maxIterations);
        params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
        LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
        result = optimizer.optimize();
    }
    VectorDynamic optComp = result.at<VectorDynamic>(key);
    cout << "Test dummy: " << optComp(0, 0) << endl;
    EndTimer("test2");
    // PrintTimer();
}

void addFactors_test(NonlinearFactorGraph &graph, VectorDynamic &expectedB, int N, char s = 'a')
{
    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    Values initialEstimateFG;
    for (int i = 0; i < N; i++)
    {
        Symbol key(s, i);
        graph.emplace_shared<test1Factor>(key, expectedB(i, 0), model);
        VectorDynamic initialEstimate = GenerateVectorDynamic(1);
        initialEstimateFG.insert(key, initialEstimate);
    }
}

TEST(graph_as_parameter, v1)
{
    int N = 10;
    VectorDynamic expectedB = GenerateVectorDynamic(N);
    for (int i = 0; i < N; i++)
        expectedB(i, 0) = sin(i);

    NonlinearFactorGraph graph;
    addFactors_test(graph, expectedB, N, 'a');
    AssertEqualScalar(10, graph.keys().size());
    addFactors_test(graph, expectedB, N, 'b');
    AssertEqualScalar(20, graph.keys().size());
    addFactors_test(graph, expectedB, N, 'c');
    AssertEqualScalar(30, graph.keys().size());
}

TEST(elimination, v1)
{
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v52.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    EliminationForest forestInfo(tasksInfo);

    VectorDynamic initialEstimate = GenerateInitialForDAG_IndexMode(dagTasks,
                                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    initialEstimate << 0, 13, 30, 80, 93;
    NonlinearFactorGraph graph;
    // AddDAG_Factor(graph, dagTasks, tasksInfo);
    AddDBF_Factor(graph, tasksInfo);
    AddDDL_Factor(graph, tasksInfo);
    Values initialEstimateFG = GenerateInitialFG(initialEstimate, tasksInfo);
    LevenbergMarquardtParams params;
    params.setlambdaInitial(initialLambda);
    // if (debugMode >= 1)
    params.setVerbosityLM("SUMMARY");
    params.setlambdaLowerBound(lowerLambda);
    params.setlambdaUpperBound(upperLambda);
    params.setRelativeErrorTol(relativeErrorTolerance);
    params.setMaxIterations(maxIterations);
    params.setUseFixedLambdaFactor(setUseFixedLambdaFactor);
    // cout << "Log file " << params.getLogFile() << endl;
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimateFG, params);
    cout << "Optimization process in elimination-v1 test: " << endl;
    auto result = optimizer.optimize();
    VectorDynamic optComp = CollectUnitOptResult(result, tasksInfo);
    cout << "Results in elimination-v1 test: " << optComp << endl;
    cout << "Error before optimization: " << graph.error(initialEstimateFG) << endl;
    cout << "Error after optimization: " << graph.error(result) << endl;
}
TEST(sigma, v1)
{
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    EliminationForest forestInfo(tasksInfo);

    VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks,
                                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    startTimeVector << 6, 97, 5, 3, 104, 2, 0, 201;
    NonlinearFactorGraph graph;
    noiseModelSigma = 1;
    AddDDL_Factor(graph, tasksInfo);
    Values initialEstimateFG = GenerateInitialFG(startTimeVector, tasksInfo);
    double err1 = graph.error(initialEstimateFG);

    NonlinearFactorGraph graph2;
    noiseModelSigma = 2;
    AddDDL_Factor(graph2, tasksInfo);
    Values initialEstimateFG2 = GenerateInitialFG(startTimeVector, tasksInfo);
    double err2 = graph2.error(initialEstimateFG2);

    AssertEqualScalar(err1, err2 * 4, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;

    return TestRegistry::runAllTests(tr);
}