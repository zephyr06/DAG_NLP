#include <CppUnitLite/TestHarness.h>
#include "../sources/InequalifyFactor.h"
TEST(inequalityFactor, evaluateError)
{
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    InequalifyFactor1D factor(key, 10, 1, model);
    VectorDynamic a = GenerateVectorDynamic(1);
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 0);
    a << 10;
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 0);
    a << 11;
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 1);
}
TEST(inequalityFactor, Hessian)
{
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    InequalifyFactor1D factor(key, 10, 1, model);

    VectorDynamic a = GenerateVectorDynamic(1);
    MatrixDynamic h = GenerateMatrixDynamic(1, 1);
    MatrixDynamic expect = GenerateMatrixDynamic(1, 1);
    auto sth = factor.evaluateError(a, h);
    assert_equal(expect, h);

    a << 10;
    sth = factor.evaluateError(a, h);
    expect << -0.5;
    assert_equal(expect, h);

    a << 11;
    expect << -1;
    sth = factor.evaluateError(a, h);
    assert_equal(expect, h);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}