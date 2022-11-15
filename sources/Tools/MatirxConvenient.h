#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "sources/Tools/testMy.h"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;
typedef long long int LLint;
extern std::mutex mtx;

inline double min(double a, double b)
{
    if (a <= b)
        return a;
    else
        return b;
    return 0;
}
inline double max(double a, double b)
{
    if (a >= b)
        return a;
    else
        return b;
    return 0;
}
void swap(VectorDynamic &x, LLint i, LLint j);

MatrixDynamic GenerateMatrixDynamic(int m, int n);

MatrixDynamic GenerateOneMatrix(int m, int n);

template <class T>
std::vector<T> Eigen2Vector(const VectorDynamic &input);

template <class T>
VectorDynamic Vector2Eigen(const std::vector<T> &input);

inline VectorDynamic GenerateVectorDynamic(LLint N)
{
    VectorDynamic v;
    v.resize(N, 1);
    v.setZero();
    return v;
}

inline VectorDynamic GenerateVectorDynamic1D(double x)
{
    VectorDynamic res = GenerateVectorDynamic(1);
    res << x;
    return res;
}