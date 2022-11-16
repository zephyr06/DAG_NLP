#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <bits/stdc++.h>
#include <Eigen/Dense>

#include "sources/Utils/colormod.h"

inline void CoutWarning(std::string message)
{
    std::cout << Color::red << message << Color::def << std::endl;
}
inline void CoutError(std::string message)
{
    CoutWarning(message);
    throw;
}

/**
 * @brief This function always trigger throw, it just prints information
 *
 * @tparam T
 * @param expect
 * @param actual
 */
template <typename T>
inline void AssertUnEqual(T expect, T actual, int lineNumber = 0)
{
    if (lineNumber != 0)
        std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
    std::cout << "Assertion failed!" << std::endl;
    std::cout << Color::red << "EXpect is " << expect << ", while the actual is " << actual << Color::def << std::endl;
    throw;
}
void AssertEqualScalar(double expected, double actual, double tolerance = 1e-6, int lineNumber = 0);

inline void AssertBool(bool expected, bool actual, int lineNumber = 0)
{
    if (expected != actual)
        return AssertUnEqual<bool>(expected, actual, lineNumber);
}

template <typename T>
void AssertEqualVectorNoRepeat(const std::vector<T> &expected, const std::vector<T> &actual,
                               double tolerance = 1e-6, int lineNumber = 0);

template <typename T>
void AssertEqualVectorExact(const std::vector<T> &expected, const std::vector<T> &actual,
                            double tolerance = 1e-6, int lineNumber = 0);

void AssertEigenEqualVector(Eigen::Matrix<double, Eigen::Dynamic, 1> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, 1> actual, int lineNumber = 0);

void AssertEigenEqualMatrix(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> actual, int lineNumber = 0);

template <class T1, class T2>
void AssertEqualMap(std::unordered_map<T1, T2> &mExpect, std::unordered_map<T1, T2> &mActual);

double SquareError(std::vector<double> &seq);