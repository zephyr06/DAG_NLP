#pragma once

#include <algorithm>
#include <chrono>
#include <math.h>
#include <unordered_map>

#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <dirent.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/function.hpp>

#include "Parameters.h"
#include "colormod.h"
#include "testMy.h"
// #include "profilier.h"
namespace RTSS21IC_NLP {

using namespace std;
using namespace gtsam;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;

typedef long long int LLint;
typedef std::map<int, std::vector<int>> ProcessorTaskSet;
struct MappingDataStruct {
  LLint index;
  double distance;

  MappingDataStruct() : index(0), distance(0) {}
  MappingDataStruct(LLint index, double distance) : index(index), distance(distance) {}

  LLint getIndex() const { return index; }
  double getDistance() const { return distance; }
  bool notEqual(const MappingDataStruct &m1, double tolerance = 1e-3) {
    if (abs(this->index - m1.getIndex()) > tolerance || abs(this->distance - m1.getDistance()) > tolerance)
      return true;
    return false;
  }
};
std::ostream &operator<<(std::ostream &os, MappingDataStruct const &m) {
  return os << m.getIndex() << ", " << m.getDistance() << std::endl;
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
double ExtractVariable(const VectorDynamic &startTimeVector, const std::vector<LLint> &sizeOfVariables,
                       int taskIndex, int instanceIndex) {
  if (taskIndex < 0 || instanceIndex < 0 || instanceIndex > sizeOfVariables[taskIndex] - 1) {

    std::cout << Color::red << "Index Error in ExtractVariable!" << Color::def << std::endl;
    throw;
  }

  LLint firstTaskInstance = 0;
  for (int i = 0; i < taskIndex; i++) {
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
LLint IndexTran_Instance2Overall(LLint i, LLint instance_i, const std::vector<LLint> &sizeOfVariables) {
  if (instance_i < 0 || instance_i > sizeOfVariables[i])
    CoutError("Instance Index out of boundary in IndexTran_Instance2Overall");
  if (i < 0 || i > static_cast<LLint>(sizeOfVariables.size()))
    CoutError("Task Index out of boundary in IndexTran_Instance2Overall");
  LLint index = 0;
  for (LLint k = 0; k < i; k++)
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
int BigIndex2TaskIndex(LLint index, const std::vector<LLint> &sizeOfVariables) {
  int taskIndex = 0;
  int N = sizeOfVariables.size();
  while (index >= 0 && taskIndex < N) {
    index -= sizeOfVariables[taskIndex];
    taskIndex++;
  }
  return taskIndex - 1;
}

inline VectorDynamic GenerateVectorDynamic(LLint N) {
  VectorDynamic v;
  v.resize(N, 1);
  v.setZero();
  return v;
}
inline MatrixDynamic GenerateMatrixDynamic(int m, int n) {
  MatrixDynamic M;
  M.resize(m, n);
  M.setZero();
  return M;
}

template <class T> std::vector<T> Eigen2Vector(const VectorDynamic &input) {
  std::vector<T> res;
  LLint len = input.rows();
  res.reserve(len);
  for (LLint i = 0; i < len; i++)
    res.push_back(input.coeff(i, 0));
  return res;
}
template <class T> VectorDynamic Vector2Eigen(const std::vector<T> &input) {

  LLint len = input.size();
  VectorDynamic res;
  res.resize(len, 1);
  for (LLint i = 0; i < len; i++)
    res(i, 0) = input.at(i);
  return res;
}
double QuotientDouble(double a, int b) {
  double left = a - int(a);
  return left + int(a) % b;
}
namespace DAG_SPACE {
/**
 * barrier function for the optimization
 **/
double Barrier(double x) {
  if (x >= 0)
    // return pow(x, 2);
    return 0;
  else if (x < 0) {
    return -1 * x;
  }
  // else // it basically means x=0
  //     return weightLogBarrier *
  //            log(x + toleranceBarrier);
  return 0;
}

double BarrierLog(double x) {
  if (x >= 0)
    // return pow(x, 2);
    return weightLogBarrier * log(x + 1) + barrierBase;
  else if (x < 0) {
    return punishmentInBarrier * pow(1 - x, 2);
  }
  // else // it basically means x=0
  //     return weightLogBarrier *
  //            log(x + toleranceBarrier);
  return 0;
}
MatrixDynamic NumericalDerivativeDynamicUpper(boost::function<VectorDynamic(const VectorDynamic &)> h,
                                              VectorDynamic x, double deltaOptimizer, int mOfJacobian) {
  int n = x.rows();
  MatrixDynamic jacobian;
  jacobian.resize(mOfJacobian, n);
  VectorDynamic currErr = h(x);
  // if (debugMode == 1)
  // {
  //    std::cout << "currErr" << currErr <<std::endl
  //          <<std::endl;
  // }

  for (int i = 0; i < n; i++) {
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
    //    std::cout << "resPlus" << resPlus <<std::endl;
    // }

    for (int j = 0; j < mOfJacobian; j++) {
      jacobian(j, i) = (resPlus(j, 0) - resMinus(j, 0)) / 2 / deltaOptimizer;
      // jacobian(j, i) = (resMinus(j, 0) - currErr(j, 0)) / deltaOptimizer * -1;
      // jacobian(j, i) = (resPlus(j, 0) - currErr(j, 0)) / deltaOptimizer;
    }
  }
  return jacobian;
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
double GetSingleElement(LLint index, VectorDynamic &actual, const MAP_Index2Data &mapIndex,
                        std::vector<bool> &filledTable) {
  if (filledTable[index])
    return actual(index, 0);
  auto it = mapIndex.find(index);

  if (it != mapIndex.end()) {
    MappingDataStruct curr = it->second;
    actual(index, 0) = GetSingleElement(curr.getIndex(), actual, mapIndex, filledTable) + curr.getDistance();
    filledTable[index] = true;
  } else {
    CoutError("Out of boundary in GetSingleElement, RecoverStartTimeVector!");
  }
  return actual(index, 0);
}

VectorDynamic RecoverStartTimeVector(const VectorDynamic &compressed, const std::vector<bool> &maskEliminate,
                                     const MAP_Index2Data &mapIndex) {
  LLint variableDimension = maskEliminate.size();
  std::vector<bool> filledTable(variableDimension, 0);

  VectorDynamic actual = GenerateVectorDynamic(variableDimension);
  LLint index = 0;
  for (LLint i = 0; i < variableDimension; i++) {
    if (not maskEliminate[i]) {
      filledTable[i] = 1;
      actual[i] = compressed(index++, 0);
    }
  }
  for (LLint i = 0; i < variableDimension; i++) {
    if (not filledTable[i]) {
      actual(i, 0) = GetSingleElement(i, actual, mapIndex, filledTable);
    }
  }
  return actual;
}
/**
 * @brief Given an index, find the final index that it depends on;
 *
 * @param index
 * @param mapIndex
 * @return LLint
 */
LLint FindLeaf(LLint index, const MAP_Index2Data &mapIndex) {
  if (index == mapIndex.at(index).getIndex())
    return index;
  else
    return FindLeaf(mapIndex.at(index).getIndex(), mapIndex);
  return -1;
}

/**
 * @brief m maps from index in original startTimeVector to index in compressed startTimeVector
 *
 * @param maskForEliminate
 * @return std::unordered_map<LLint, LLint>
 */
std::unordered_map<LLint, LLint> MapIndex_True2Compress(const std::vector<bool> &maskForEliminate) {

  std::unordered_map<LLint, LLint> m;
  // count is the index in compressed startTimeVector
  int count = 0;
  for (size_t i = 0; i < maskForEliminate.size(); i++) {
    if (maskForEliminate.at(i) == false)
      m[i] = count++;
  }
  return m;
}

inline void UpdateSM(double val, LLint i, LLint j, SM_Dynamic &sm) {
  // if (sm.coeffRef(i, j))
  sm.coeffRef(i, j) = val;
  // else
  // {
  //     sm.insert(i, j) = val;
  // }
}

/**
 * @brief generate analytic Jacobian for elimination part
 *
 * @param length the number of all the variables
 * @param sizeOfVariables
 * @param mapIndex encode elimination relationship
 * @param mapIndex_True2Compress
 * @return j_map: (length, lengthCompressed)
 * a sparse matrix that represents Jacobian matrix of compreseed variables w.r.t. original variables
 */
SM_Dynamic JacobianElimination(LLint length, LLint lengthCompressed,
                               const std::vector<LLint> &sizeOfVariables, const MAP_Index2Data &mapIndex,
                               const std::unordered_map<LLint, LLint> &mapIndex_True2Compress) {
  SM_Dynamic j_map(length, lengthCompressed);
  // go through all the variables
  for (int i = 0; i < int(sizeOfVariables.size()); i++) {
    for (int j = 0; j < int(sizeOfVariables.at(i)); j++) {
      LLint bigIndex = IndexTran_Instance2Overall(i, j, sizeOfVariables);
      // find its final dependency variable
      LLint finalIndex = FindLeaf(bigIndex, mapIndex);
      j_map.insert(bigIndex, mapIndex_True2Compress.at(finalIndex)) = 1;
    }
  }
  return j_map;
}
} // namespace DAG_SPACE
} // namespace RTSS21IC_NLP