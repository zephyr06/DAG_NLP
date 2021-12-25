#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

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
    PrintTimer();
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
using namespace DAG_SPACE;
double GraphErrorEvaluation(DAG_Model &dagTasks, VectorDynamic startTimeVector,
                            NonlinearFactorGraph &graph)
{
    TaskSet tasks = dagTasks.tasks;
    TaskSetInfoDerived tasksInfo(tasks);
    EliminationForest forestInfo(tasksInfo);
    Symbol key('a', 0);
    ;
    // LLint errorDimensionMS = 1;

    LLint errorDimensionDAG = dagTasks.edgeNumber();
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
    graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, tasksInfo, forestInfo,
                                               errorDimensionDAG, model);
    if (makespanWeight > 0)
    {
        LLint errorDimensionMS = 1;
        model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
        graph.emplace_shared<MakeSpanFactor>(key, dagTasks, tasksInfo, forestInfo,
                                             errorDimensionMS, model);
    }

    ProcessorTaskSet processorTaskSet = ExtractProcessorTaskSet(dagTasks.tasks);
    LLint errorDimensionDBF = processorTaskSet.size();
    model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
    graph.emplace_shared<DBF_ConstraintFactor>(key, tasksInfo, forestInfo,
                                               errorDimensionDBF,
                                               model);

    LLint errorDimensionDDL = 2 * tasksInfo.variableDimension;
    model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
    graph.emplace_shared<DDL_ConstraintFactor>(key, tasksInfo, forestInfo,
                                               errorDimensionDDL, model);

    if (weightPrior_factor > 0)
    {
        LLint errorDimensionPrior = 1;
        vector<int> order = FindDependencyOrder(dagTasks);
        model = noiseModel::Isotropic::Sigma(errorDimensionPrior, noiseModelSigma);
        graph.emplace_shared<Prior_ConstraintFactor>(key, tasksInfo, forestInfo,
                                                     errorDimensionPrior, 0.0, order[0], model);
    }

    // LLint errorDimensionSF = CountSFError(dagTasks, sizeOfVariables);
    // model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
    // graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
    //                                                     errorDimensionSF, sensorFusionTolerance,
    //                                                     mapIndex, maskForEliminate, model);
    Values initialEstimateFG;
    // Symbol key('a', 0);
    initialEstimateFG.insert(key, startTimeVector);
    return graph.error(initialEstimateFG);
}
TEST(pass_graph_as_parameter, v1)
{
    auto dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v17.csv", "orig");
    TaskSetInfoDerived tasksInfo(dagTasks.tasks);
    EliminationForest forestInfo(tasksInfo);

    VectorDynamic startTimeVector = GenerateInitialForDAG_IndexMode(dagTasks,
                                                                    tasksInfo.sizeOfVariables, tasksInfo.variableDimension);
    NonlinearFactorGraph graph;
    double expected = GraphErrorEvaluation(dagTasks, startTimeVector, graph);
    Symbol key('a', 0);
    Values initialEstimateFG;
    initialEstimateFG.insert(key, startTimeVector);
    double actual = graph.error(initialEstimateFG);

    AssertEqualScalar(expected, actual);
}

int main()
{
    BeginTimer("main");
    TestResult tr;

    auto a = TestRegistry::runAllTests(tr);
    EndTimer("main");
    return a;
}