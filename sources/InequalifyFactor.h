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
