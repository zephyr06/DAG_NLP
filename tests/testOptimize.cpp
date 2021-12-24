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
TEST(somefactor, v1)
{

    // build the factor graph
    NonlinearFactorGraph graph;
    Symbol key('a', 0);

    LLint errorDimensionMS = 1;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionMS, noiseModelSigma);
    graph.emplace_shared<test1Factor>(key, 0, model);
    VectorDynamic initialEstimate = GenerateVectorDynamic(1);
    initialEstimate << 10;
    Values initialEstimateFG;
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
    cout << "Test dummy: " << optComp << endl;
}

// **********************************************************
// TEST(RandomWalk, v1)
// {
//     using namespace DAG_SPACE;
//     using namespace RegularTaskSystem;

//     DAG_SPACE::DAG_Model dagTasks = ReadDAG_Tasks("../../TaskData/test_n5_v9.csv", "orig");
//     TaskSet tasks = dagTasks.tasks;
//     int N = tasks.size();
//     LLint hyperPeriod = HyperPeriod(tasks);

//     // declare variables
//     vector<LLint> sizeOfVariables;
//     int variableDimension = 0;
//     for (int i = 0; i < N; i++)
//     {

//         LLint size = hyperPeriod / tasks[i].period;
//         sizeOfVariables.push_back(size);
//         variableDimension += size;
//     }
//     pair<Graph, indexVertexMap> sth = EstablishGraphStartTimeVector(dagTasks);
//     Graph eliminationTrees = sth.first;
//     indexVertexMap indexesBGL = sth.second;
//     VectorDynamic initial = GenerateInitial(dagTasks, sizeOfVariables, variableDimension);
//     initial << 6, 0, 1, 2, 5;
//     VectorDynamic actual = RandomWalk(initial, dagTasks, eliminationTrees, indexesBGL);
//     VectorDynamic expect = initial;
//     expect << 3, 0, 1, 2, 5;
//     assert_equal(expect, actual);
//     AssertEigenEqualVector(expect, actual);
// }
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}