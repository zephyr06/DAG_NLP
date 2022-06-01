#include <CppUnitLite/TestHarness.h>
#include "sources/Factors/InequalifyFactor.h"
TEST(SmallerThanFactor1D, evaluateError)
{
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    SmallerThanFactor1D factor(key, 10, model);
    VectorDynamic a = GenerateVectorDynamic(1);
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 0);
    a << 10;
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 0);
    a << 11;
    AssertEqualScalar(factor.evaluateError(a)(0, 0), 1);
}
TEST(LargerThanFactor1D, evaluateError)
{
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    LargerThanFactor1D factor(key, 10, model);
    VectorDynamic a = GenerateVectorDynamic(1);
    AssertEqualScalar(10, factor.evaluateError(a)(0, 0), 1e-3, __LINE__);
    a << 10;
    AssertEqualScalar(0, factor.evaluateError(a)(0, 0), 1e-3, __LINE__);
    a << 11;
    AssertEqualScalar(0, factor.evaluateError(a)(0, 0), 1e-3, __LINE__);
}
TEST(SmallerThanFactor1D, Hessian)
{
    boost::function<VectorDynamic(const VectorDynamic &)> f =
        [this](const VectorDynamic &x)
    {
        Symbol key('a', 0);

        auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        SmallerThanFactor1D factor(key, 10, model);
        return factor.evaluateError(x);
    };
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    SmallerThanFactor1D factor(key, 10, model);

    VectorDynamic a = GenerateVectorDynamic(1);
    MatrixDynamic actual = GenerateMatrixDynamic(1, 1);
    auto sth = factor.evaluateError(a, actual);
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);

    a << 10;
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);

    a << 10.01;
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);

    a << 11;
    actual = GenerateMatrixDynamic(1, 1);
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);
}
TEST(LargerThanFactor1D, Hessian)
{
    boost::function<VectorDynamic(const VectorDynamic &)> f =
        [this](const VectorDynamic &x)
    {
        Symbol key('a', 0);

        auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
        LargerThanFactor1D factor(key, 10, model);
        return factor.evaluateError(x);
    };
    Symbol key('a', 0);

    auto model = noiseModel::Isotropic::Sigma(1, noiseModelSigma);
    LargerThanFactor1D factor(key, 10, model);

    VectorDynamic a = GenerateVectorDynamic(1);
    MatrixDynamic actual = GenerateMatrixDynamic(1, 1);
    auto sth = factor.evaluateError(a, actual);
    MatrixDynamic expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);

    a << 10;
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);
    a << 10.01;
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);

    a << 11;
    actual = GenerateMatrixDynamic(1, 1);
    sth = factor.evaluateError(a, actual);
    expect = NumericalDerivativeDynamicUpper(f, a, deltaOptimizer, 1);
    assert_equal(expect, actual);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}