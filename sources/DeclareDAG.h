#pragma once

#include <chrono>
#include <unordered_map>
#include <math.h>

#include <Eigen/Dense>
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

#include "Parameters.h"

using namespace std;
using namespace gtsam;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef long long int LLint;
struct MappingDataStruct
{
    LLint index;
    double distance;

    double getIndex() const
    {
        return index;
    }
    double getDistance() const
    {
        return distance;
    }
};
// typedef unordered_map<int, boost::function<double(const VectorDynamic &, int)>> MAP_Index2Func;
typedef unordered_map<int, MappingDataStruct> MAP_Index2Data;
typedef boost::function<VectorDynamic(const VectorDynamic &)> FuncV2V;

/**
 * @brief decode mapping; used in RecoverStartTimeVector, 
 * given compressed startTimeVector and one index in original startTimeVector,
 * return the true value of original startTimeVector[index]
 * 
 * @param startTimeVector 
 * @param index 
 * @param m 
 * @return double 
 */
inline double DecodeMap(const VectorDynamic &startTimeVector, const LLint index, MAP_Index2Data m)
{
    return startTimeVector(m[index].getIndex(), 0) + m[index].getDistance();
}

// ************************************************************ SOME FUNCTIONS
/**
     * @brief Given a task index and instance index, return its start time in startTimeVector
     * 
     * @param startTimeVector large, combined vector of start time for all instances
     * @param taskIndex task-index
     * @param instanceIndex instance-index
     * @return double start time of the extracted instance
     */
double ExtractVariable(const VectorDynamic &startTimeVector,
                       const vector<LLint> &sizeOfVariables,
                       int taskIndex, int instanceIndex)
{
    if (taskIndex < 0 || instanceIndex < 0)
    {
        cout << red << "Index Error in ExtractVariable!" << def << endl;
        throw;
    }
    LLint firstTaskInstance = 0;
    for (int i = 0; i < taskIndex; i++)
    {
        firstTaskInstance += sizeOfVariables[i];
    }
    return startTimeVector(firstTaskInstance + instanceIndex, 0);
}

/**
 * @brief given task index and instance index, return variable's index in StartTimeVector
 * 
 * @param i 
 * @param instance_i 
 * @param sizeOfVariables 
 * @return LLint 
 */
LLint IndexTran_Instance2Overall(LLint i, LLint instance_i, const vector<LLint> &sizeOfVariables)
{
    LLint index = 0;
    for (size_t k = 0; k < i; k++)
        index += sizeOfVariables[k];
    return index + instance_i;
}

/**
 * @brief Given index in startTimeVector, decode its task index
 * 
 * @param index 
 * @param sizeOfVariables 
 * @return int: task index
 */
int BigIndex2TaskIndex(LLint index, const vector<LLint> &sizeOfVariables)
{
    int taskIndex = 0;
    int N = sizeOfVariables.size();
    while (index >= 0 && taskIndex < N)
    {
        index -= sizeOfVariables[taskIndex];
        taskIndex++;
    }
    return taskIndex - 1;
}

inline VectorDynamic GenerateVectorDynamic(LLint N)
{
    VectorDynamic v;
    v.resize(N, 1);
    v.setZero();
    return v;
}

namespace DAG_SPACE
{
    /**
 * barrier function for the optimization
 **/
    double Barrier(double x)
    {
        if (x >= 0)
            // return pow(x, 2);
            return 0;
        else if (x < 0)
        {
            return -1 * x;
        }
        else // it basically means x=0
            return weightLogBarrier *
                   log(x + toleranceBarrier);
    }

    double BarrierLog(double x)
    {
        if (x >= 0)
            // return pow(x, 2);
            return weightLogBarrier * log(x + 1) + barrierBase;
        else if (x < 0)
        {
            return punishmentInBarrier * pow(1 - x, 2);
        }
        else // it basically means x=0
            return weightLogBarrier *
                   log(x + toleranceBarrier);
    }

    MatrixDynamic NumericalDerivativeDynamicUpper(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                                  VectorDynamic x, double deltaOptimizer, int mOfJacobian)
    {
        int n = x.rows();
        MatrixDynamic jacobian;
        jacobian.resize(mOfJacobian, n);
        VectorDynamic currErr = h(x);
        // if (debugMode == 1)
        // {
        //     cout << "currErr" << currErr << endl
        //          << endl;
        // }

        for (int i = 0; i < n; i++)
        {
            VectorDynamic xDelta = x;
            xDelta(i, 0) = xDelta(i, 0) + deltaOptimizer;
            VectorDynamic resPlus;
            resPlus.resize(mOfJacobian, 1);
            resPlus = h(xDelta);
            xDelta(i, 0) = xDelta(i, 0) - 2 * deltaOptimizer;
            VectorDynamic resMinus;
            resMinus.resize(mOfJacobian, 1);
            resMinus = h(xDelta);
            // if (debugMode == 1)
            // {
            //     cout << "resPlus" << resPlus << endl;
            // }

            for (int j = 0; j < mOfJacobian; j++)
            {
                jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
                // jacobian(j, i) = (resMinus(j, 0) - currErr(j, 0)) / deltaOptimizer * -1;
                // jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
            }
        }
        return jacobian;
    }

    VectorDynamic RecoverStartTimeVector(const VectorDynamic &compressed,
                                         const vector<bool> &maskEliminate,
                                         const MAP_Index2Data &mapIndex)
    {
        LLint variableDimension = maskEliminate.size();
        VectorDynamic actual = GenerateVectorDynamic(variableDimension);
        size_t index = 0;
        for (size_t i = 0; i < variableDimension; i++)
        {
            if (maskEliminate[i])
            {
                actual(i, 0) = DecodeMap(actual, i, mapIndex);
            }
            else
                actual(i, 0) = compressed(index++, 0);
        }
        return actual;
    }

}