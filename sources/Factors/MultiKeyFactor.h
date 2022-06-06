#pragma once

#pragma once

#include <chrono>
#include <unordered_map>
#include <math.h>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <dirent.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/function.hpp>

#include "sources/Utils/Parameters.h"
#include "sources/Utils/DeclareDAG.h"
#include "sources/Tools/colormod.h"
#include "sources/Tools/testMy.h"
#include "sources/Tools/profilier.h"

using namespace std;
using namespace gtsam;

// typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
// typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;
// typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
// typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;
typedef boost::function<Vector(const Values &x)> LambdaMultiKey;

typedef long long int LLint;

// using namespace DAG_SPACE;
typedef std::vector<VectorDynamic> VVec;

class MultiKeyFactor : public NoiseModelFactor
{
public:
    vector<Symbol> keyVec;
    uint keySize;
    uint mOfJacobian;
    LambdaMultiKey lambdaMK;

    MultiKeyFactor(vector<Symbol> keyVec, LambdaMultiKey lambdaMK, uint mOfJacobian,
                   SharedNoiseModel model) : NoiseModelFactor(model, keyVec),
                                             keyVec(keyVec),
                                             keySize(keyVec.size()), mOfJacobian(mOfJacobian),
                                             lambdaMK(lambdaMK)
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
            for (uint i = 0; i < keySize; i++)
            {
                NormalErrorFunction1D f =
                    [x, i, this](const VectorDynamic xi)
                {
                    Symbol a = keyVec.at(i);
                    Values xx = x;
                    xx.update(a, xi);
                    return lambdaMK(xx);
                };
                (*H)[i] = NumericalDerivativeDynamic(f, x.at<VectorDynamic>(keyVec[i]), deltaOptimizer, mOfJacobian);
            }
        }
        return lambdaMK(x);
    }
};