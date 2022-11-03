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
#include "sources/Tools/colormod.h"
#include "sources/Tools/testMy.h"
#include "sources/Tools/profilier.h"
#include "sources/Tools/MatirxConvenient.h"

using namespace std;
// TODO: remove using namespace
using namespace gtsam;

typedef boost::function<VectorDynamic(const VectorDynamic &)> NormalErrorFunction1D;
typedef boost::function<VectorDynamic(const VectorDynamic &, const VectorDynamic &)> NormalErrorFunction2D;
typedef boost::function<Vector(const Values &x)> LambdaMultiKey;

typedef long long int LLint;

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
    // BeginTimerAppInProfiler;
    if (instance_i < 0 || instance_i > sizeOfVariables[i])
        CoutError("Instance Index out of boundary in IndexTran_Instance2Overall");
    if (i < 0 || i > (LLint)sizeOfVariables.size())
        CoutError("Task Index out of boundary in IndexTran_Instance2Overall");
    LLint index = 0;
    for (size_t k = 0; k < (size_t)i; k++)
        index += sizeOfVariables[k];
    // EndTimerAppInProfiler;
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

/**
 * @brief generate key symbol for use in GTSAM
 *
 * @param idtask task index
 * @param j job index
 * @return gtsam::Symbol
 */
inline gtsam::Symbol GenerateKey(int idtask, LLint j)
{
    if (idtask > 100 || j > 100000 || idtask < 0 || j < 0)
    {
        CoutWarning("Invalid index in GenerateKey!");
    }
    gtsam::Symbol key('a' + idtask, j);
    return key;
}

// return (taskId, jobId)
inline pair<int, LLint> AnalyzeKey(gtsam::Symbol key)
{
    int id = key.chr() - 'a';
    LLint index = key.index();
    return make_pair(id, index);
}
// inline std::vector<gtsam::Symbol> GenerateKey(std::vector<int> &idtask,
//                                               std::vector<LLint> &index)
// {

//     AssertEqualScalar(idtask.size(), index.size(), 1e-6, __LINE__);
//     std::vector<gtsam::Symbol> res;
//     res.reserve(idtask.size());
//     for (uint i = 0; i < idtask.size(); i++)
//     {
//         res.push_back(GenerateKey(idtask[i], index[i]));
//     }
//     return res;
// }

struct MappingDataStruct
{
    LLint index;
    double distance;

    MappingDataStruct() : index(0), distance(0) {}
    MappingDataStruct(LLint index, double distance) : index(index), distance(distance) {}

    LLint getIndex() const
    {
        return index;
    }
    double getDistance() const
    {
        return distance;
    }
    bool notEqual(const MappingDataStruct &m1, double tolerance = 1e-3)
    {
        if (abs(this->index - m1.getIndex()) > tolerance ||
            abs(this->distance - m1.getDistance()) > tolerance)
            return true;
        return false;
    }
};
std::ostream &operator<<(std::ostream &os, MappingDataStruct const &m)
{
    return os << m.getIndex() << ", " << m.getDistance() << endl;
}

// typedef unordered_map<int, boost::function<double(const VectorDynamic &, int)>> MAP_Index2Func;
typedef std::unordered_map<int, MappingDataStruct> MAP_Index2Data;
typedef boost::function<VectorDynamic(const VectorDynamic &)> FuncV2V;

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
    if (taskIndex < 0 || instanceIndex < 0 || instanceIndex > sizeOfVariables[taskIndex] - 1)
    {

        cout << Color::red << "Index Error in ExtractVariable!" << Color::def << endl;
        throw;
    }

    LLint firstTaskInstance = 0;
    for (int i = 0; i < taskIndex; i++)
    {
        firstTaskInstance += sizeOfVariables[i];
    }
    return startTimeVector(firstTaskInstance + instanceIndex, 0);
}

VectorDynamic CompresStartTimeVector(const VectorDynamic &startTimeComplete,
                                     const vector<bool> &maskForEliminate)
{
    int variableDimension = maskForEliminate.size();
    vector<double> initialUpdateVec;
    initialUpdateVec.reserve(variableDimension - 1);

    for (int i = 0; i < variableDimension; i++)
    {
        if (not maskForEliminate[i])
        {
            initialUpdateVec.push_back(startTimeComplete(i, 0));
        }
    }
    VectorDynamic initialUpdate;
    initialUpdate.resize(initialUpdateVec.size(), 1);
    for (size_t i = 0; i < initialUpdateVec.size(); i++)
    {
        initialUpdate(i, 0) = initialUpdateVec[i];
    }
    return initialUpdate;
}

double QuotientDouble(double a, int b)
{
    double left = a - int(a);
    return left + int(a) % b;
}

/**
 * barrier function for the optimization
 **/
double
Barrier(double x)
{
    if (x >= 0)
        // return pow(x, 2);
        return 0;
    else if (x < 0)
    {
        return -1 * x;
    }
    // else // it basically means x=0
    //     return weightLogBarrier *
    //            log(x + toleranceBarrier);
    return 0;
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
    // else // it basically means x=0
    //     return weightLogBarrier *
    //            log(x + toleranceBarrier);
    return 0;
}
MatrixDynamic NumericalDerivativeDynamic(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                         const VectorDynamic &x, double deltaOptimizer, int mOfJacobian)
{
    int n = x.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);
    // VectorDynamic currErr = h(x);

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

        for (int j = 0; j < mOfJacobian; j++)
        {
            jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
            // jacobian(j, i) = (resMinus(j, 0) - currErr(j, 0)) / deltaOptimizer * -1;
            // jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
        }
    }
    return jacobian;
}

MatrixDynamic NumericalDerivativeDynamic2D1(NormalErrorFunction2D h,
                                            const VectorDynamic &x1,
                                            const VectorDynamic &x2,
                                            double deltaOptimizer,
                                            int mOfJacobian)
{
    int n = x1.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);
    NormalErrorFunction1D f = [h, x2](const VectorDynamic &x1)
    {
        return h(x1, x2);
    };

    return NumericalDerivativeDynamic(f, x1, deltaOptimizer, mOfJacobian);
}
MatrixDynamic NumericalDerivativeDynamic2D2(NormalErrorFunction2D h,
                                            const VectorDynamic &x1,
                                            const VectorDynamic &x2,
                                            double deltaOptimizer,
                                            int mOfJacobian)
{
    int n = x2.rows();
    MatrixDynamic jacobian;
    jacobian.resize(mOfJacobian, n);
    NormalErrorFunction1D f = [h, x1](const VectorDynamic &x2)
    {
        return h(x1, x2);
    };

    return NumericalDerivativeDynamic(f, x2, deltaOptimizer, mOfJacobian);
}

/**
 * @brief helper function for RecoverStartTimeVector
 *
 * @param index
 * @param actual
 * @param mapIndex
 * @param filledTable
 * @return double
 */
double GetSingleElement(LLint index, VectorDynamic &actual,
                        const MAP_Index2Data &mapIndex,
                        vector<bool> &filledTable)
{
    if (filledTable[index])
        return actual(index, 0);
    auto it = mapIndex.find(index);

    if (it != mapIndex.end())
    {
        MappingDataStruct curr = it->second;
        actual(index, 0) = GetSingleElement(curr.getIndex(), actual,
                                            mapIndex, filledTable) +
                           curr.getDistance();
        filledTable[index] = true;
    }
    else
    {
        CoutError("Out of boundary in GetSingleElement, RecoverStartTimeVector!");
    }
    return actual(index, 0);
}

inline void UpdateSM(double val, LLint i, LLint j, SM_Dynamic &sm)
{
    // if (sm.coeffRef(i, j))
    sm.coeffRef(i, j) = val;
    // else
    // {
    //     sm.insert(i, j) = val;
    // }
}

/**
 * @brief generate a random number within the range [a,b];
 * a must be smaller than b
 *
 * @param a
 * @param b
 * @return double
 */
inline double RandRange(double a, double b)
{
    if (b < a)
    {
        // CoutError("Range Error in RandRange");
        return a + (b - a) * double(rand()) / RAND_MAX;
    }
    return a + (b - a) * double(rand()) / RAND_MAX;
}

void AddEntry(std::string pathRes, std::string s)
{
    std::ofstream outfileWrite;
    outfileWrite.open(pathRes,
                      std::ios_base::app);
    outfileWrite << s << std::endl;
    outfileWrite.close();
}

std::string ResizeStr(std::string s, uint size = 20)
{
    if (s.size() > size)
    {
        return s.substr(0, size);
    }
    else
    {
        std::string res = "";
        for (size_t i = 0; i < size - s.size(); i++)
        {
            res += " ";
        }
        return res + s;
    }
}
