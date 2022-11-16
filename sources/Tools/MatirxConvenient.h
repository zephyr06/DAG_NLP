#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixDynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorDynamic;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SM_Dynamic;
typedef long long int LLint;

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

// template <class T>
// std::vector<T> Eigen2Vector(const VectorDynamic &input);

// TODO: figure out why can't separate it to .cpp
template <class T>
inline std::vector<T> Eigen2Vector(const VectorDynamic &input)
{
    std::vector<T> res;
    LLint len = input.rows();
    res.reserve(len);
    for (LLint i = 0; i < len; i++)
        res.push_back(input.coeff(i, 0));
    return res;
}
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