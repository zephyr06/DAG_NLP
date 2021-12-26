#pragma once
#include "DeclareDAG.h"
/**
 * @brief Constraint of x <= b
 * 
 */
class SmallerThanFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    double b;
    double weight;
    SmallerThanFactor1D(Key key, double b,
                        double weight,
                        SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                  b(b), weight(weight) {}

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        AssertEqualScalar(1, x.rows(), 1e-6, __LINE__);
        VectorDynamic err = GenerateVectorDynamic(1);
        double err1 = b - x(0, 0);
        err(0, 0) = Barrier(err1);
        if (H)
        {
            MatrixDynamic hh = GenerateMatrixDynamic(1, 1);
            if (err1 <= -deltaOptimizer)
            {
                hh(0, 0) = 1 * weight;
            }
            else if (err1 <= deltaOptimizer)
            {
                double errP1 = Barrier(err1 - deltaOptimizer);
                double errM1 = Barrier(err1 + deltaOptimizer);
                hh(0, 0) = (errP1 - errM1) / 2 / deltaOptimizer * weight;
            }
            // else, hh(0,0)=0

            *H = hh;
        }
        return err;
    }
};

/**
 * @brief Constraint of x >= b
 * 
 */
class LargerThanFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    double b;
    double weight;
    LargerThanFactor1D(Key key, double b,
                       double weight,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 b(b), weight(weight) {}

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        AssertEqualScalar(1, x.rows(), 1e-6, __LINE__);
        VectorDynamic err = GenerateVectorDynamic(1);
        double err1 = x(0, 0) - b;
        err(0, 0) = Barrier(err1) * weight;
        if (H)
        {
            MatrixDynamic hh = GenerateMatrixDynamic(1, 1);
            if (err1 <= -deltaOptimizer)
            {
                hh(0, 0) = -1 * weight;
            }
            else if (err1 <= deltaOptimizer)
            {
                double errP1 = Barrier(err1 + deltaOptimizer);
                double errM1 = Barrier(err1 - deltaOptimizer);
                hh(0, 0) = (errP1 - errM1) / 2 / deltaOptimizer * weight;
            }
            // else, hh(0,0)=0

            *H = hh;
        }
        return err;
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
    double weight;
    NormalErrorFunction2D f;
    InequalityFactor2D(Key key1, Key key2,
                       double weight,
                       SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2),
                                                 weight(weight)
    {
        // an example of the f function
        f = [weight](const VectorDynamic &x1, const VectorDynamic &x2)
        {
            return (x1 + x2) * weight;
        };
    }

    InequalityFactor2D(Key key1, Key key2,
                       double weight, NormalErrorFunction2D f,
                       SharedNoiseModel model) : NoiseModelFactor2<VectorDynamic, VectorDynamic>(model, key1, key2),
                                                 weight(weight), f(f)
    {
    }

    Vector evaluateError(const VectorDynamic &x1, const VectorDynamic &x2,
                         boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none) const override
    {
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
        return err;
    }
};