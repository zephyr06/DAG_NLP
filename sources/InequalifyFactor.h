#pragma once
#include "DeclareDAG.h"
/**
 * @brief Constraint of x <= b
 * 
 */
class InequalifyFactor1D : public NoiseModelFactor1<VectorDynamic>
{
public:
    double b;
    double weight;
    InequalifyFactor1D(Key key, double b,
                       double weight,
                       SharedNoiseModel model) : NoiseModelFactor1<VectorDynamic>(model, key),
                                                 b(b), weight(weight) {}

    Vector evaluateError(const VectorDynamic &x,
                         boost::optional<Matrix &> H = boost::none) const override
    {
        AssertEqualScalar(1, x.rows(), 1e-6, __LINE__);
        VectorDynamic err = GenerateVectorDynamic(1);
        double err1 = Barrier(b - x(0, 0)) * weight;
        err(0, 0) = err1;
        if (H)
        {
            MatrixDynamic hh = GenerateMatrixDynamic(1, 1);
            if (err(0, 0) <= -deltaOptimizer)
            {
                hh(0, 0) = 1 * weight;
            }
            else if (err(0, 0) <= deltaOptimizer)
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
