#include "../sources/Optimize.h"
#include "../sources/testMy.h"
#include "../sources/EliminationForest_utils.h"

#include "../sources/MultiKeyFactor.h"

TEST(MultiKeyFactor, v1)
{
    NonlinearFactorGraph graph;
    uint dimension = 2;
    LLint errorDimensionDBF = dimension;
    auto model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);

    Symbol key0('a', 0);
    Symbol key1('a', 1);
    vector<Symbol> keyVec = {key0, key1};
    vector<double> bv = {5, 6};
    LambdaMultiKey f = [bv, keyVec, dimension](const Values &x)
    {
        // const VectorDynamic &x0 = x.at<VectorDynamic>(keyVec[0]);
        // const VectorDynamic &x1 = x.at<VectorDynamic>(keyVec[1]);

        VectorDynamic res = GenerateVectorDynamic(dimension);
        for (uint i = 0; i < dimension; i++)
        {
            res(i, 0) = bv[i] - x.at<VectorDynamic>(keyVec[i])(0, 0);
        }
        return res;
    };
    MultiKeyFactor factor(keyVec, f, model);

    Values initialEstimateFG;
    VectorDynamic x0 = GenerateVectorDynamic(1);
    VectorDynamic x1 = GenerateVectorDynamic(1);
    initialEstimateFG.insert(key0, x0);
    initialEstimateFG.insert(key1, x1);
    VectorDynamic res = factor.unwhitenedError(initialEstimateFG);
    VectorDynamic expect = GenerateVectorDynamic(2);
    expect << 5, 6;
    AssertEigenEqualVector(expect, res, __LINE__);

    std::vector<MatrixDynamic> Hs, HsExpect;
    Hs.reserve(dimension);
    // for (uint i = 0; i < dimension; i++)
    // {
    MatrixDynamic m = GenerateMatrixDynamic(2, 1);
    Hs.push_back(m);
    Hs.push_back(m);
    m << -1, 0;
    HsExpect.push_back(m);
    m << 0, -1;
    HsExpect.push_back(m);
    // }
    res = factor.unwhitenedError(initialEstimateFG, Hs);
    for (uint i = 0; i < dimension; i++)
    {
        assert_equal(HsExpect[i], Hs[i]);
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}