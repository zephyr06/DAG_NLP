#pragma once

#include "DeclareDAG.h"

using namespace DAG_SPACE;
typedef std::vector<VectorDynamic> VVec;

class MultiKeyFactor : public NoiseModelFactor
{
public:
    vector<Symbol> keyVec;
    uint dimension;
    LambdaMultiKey lambdaMK;

    MultiKeyFactor(vector<Symbol> keyVec, LambdaMultiKey lambdaMK,
                   SharedNoiseModel model) : NoiseModelFactor(model, keyVec),
                                             keyVec(keyVec),
                                             dimension(keyVec.size()), lambdaMK(lambdaMK)
    {
    }

    Vector unwhitenedError(const Values &x,
                           boost::optional<std::vector<Matrix> &> H = boost::none) const override
    {
        // const VectorDynamic &x0 = x.at<VectorDynamic>(keyVec[0]);
        // const VectorDynamic &x1 = x.at<VectorDynamic>(keyVec[1]);

        // VectorDynamic res = GenerateVectorDynamic(dimension);
        // for (uint i = 0; i < dimension; i++)
        // {
        //     res(i, 0) = bv[i] - x.at<VectorDynamic>(keyVec[i])(0, 0);
        // }
        // cout << "Res: " << res << endl;
        if (H)
        {
            for (uint i = 0; i < dimension; i++)
            {
                NormalErrorFunction1D f =
                    [x, i, this](const VectorDynamic xi)
                {
                    Symbol a = keyVec.at(i);
                    Values xx = x;
                    xx.update(a, xi);
                    return lambdaMK(xx);
                };
                (*H)[i] = NumericalDerivativeDynamicUpper(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer, 2);
            }
        }
        return lambdaMK(x);
    }
};