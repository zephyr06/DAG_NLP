#pragma once
#include "sources/Utils/DeclareDAG.h"

/**
 * @brief Constraint of x <= b
 *
 */
class InequalityFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    NormalErrorFunction1D f;
    int dimension;
    /**
     * @brief Construct a new Inequality Factor 1 D object,
     *  mainly used in derived class because f is not defined
     */
    InequalityFactor1D(Key key,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key)
    {
        dimension = 1;
    }

    InequalityFactor1D(Key key, NormalErrorFunction1D f,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 f(f)
    {
        dimension = 1;
    }

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        BeginTimer("InequalityFactor1D");
        // AssertEqualScalar(1, x.rows(), 1e-6, __LINE__);
        VectorDynamic err = f(x);
        if (H)
        {
            *H = NumericalDerivativeDynamic(f, x, deltaOptimizer, err.rows());
        }
        EndTimer("InequalityFactor1D");
        return err;
    }
};

/**
 * @brief Constraint of x >= b
 *
 */
class SmallerThanFactor1D : public InequalityFactor1D
{
public:
    double b;
    SmallerThanFactor1D(Key key, double b,
                        SharedNoiseModel model) : InequalityFactor1D(key, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << Barrier(b - x(0, 0));
            return res;
        };
    }
};

/**
 * @brief Constraint of x >= b
 *
 */
class LargerThanFactor1D : public InequalityFactor1D
{
public:
    double b;
    LargerThanFactor1D(Key key, double b,
                       SharedNoiseModel model) : InequalityFactor1D(key, model)
    {
        f = [b](const VectorDynamic &x)
        {
            VectorDynamic res = x;
            res << Barrier(x(0, 0) - b);
            return res;
        };
    }
};

/**
 * @brief Constraint of f(x1, x2) <= 0;
 * x1 and x2 are vectors of size (1,1)
 *
 */
class InequalityFactor2D : public NoiseModelFactor2<VectorDynamic, VectorDynamic>
{
public:
    NormalErrorFunction2D f;
    InequalityFactor2D(Key key1, Key key2,
                       SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2)
    {
        // an example of the f function
        f = [](const VectorDynamic &x1, const VectorDynamic &x2)
        {
            return (x1 + x2);
        };
    }

    InequalityFactor2D(Key key1, Key key2, NormalErrorFunction2D f,
                       SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2),
                                                 f(f)
    {
    }

    Vector evaluateError(const VectorDynamic &x1, const VectorDynamic &x2,
                         boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const override
    {
        BeginTimer("InequalityFactor2D");
        // AssertEqualScalar(1, x.rows(), 1e-6, __LINE__);
        VectorDynamic err = f(x1, x2);
        if (H1)
        {
            *H1 = NumericalDerivativeDynamic2D1(f, x1, x2, deltaOptimizer, 1);
        }
        if (H2)
        {
            *H2 = NumericalDerivativeDynamic2D2(f, x1, x2, deltaOptimizer, 1);
        }
        EndTimer("InequalityFactor2D");
        return err;
    }
};