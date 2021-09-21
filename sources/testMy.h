#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
#include <Eigen/Dense>

#include "colormod.h"

using namespace std;

void CoutWarning(string message)
{
    cout << Color::red << message << Color::def << endl;
}
void CoutError(string message)
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
void AssertUnEqual(T expect, T actual)
{
    std::cout << "Assertion failed!" << std::endl;
    cout << Color::red << "EXpect is " << expect << ", while the actual is " << actual << Color::def << endl;
    throw;
}
void AssertEqualScalar(double expected, double actual, double tolerance = 1e-6)
{
    if (abs(expected - actual) < tolerance)
        return;
    else
    {
        AssertUnEqual<double>(expected, actual);
    }
}

void AssertBool(bool expected, bool actual)
{
    if (expected != actual)
        return AssertUnEqual<bool>(expected, actual);
}

template <typename T>
void AssertEqualVectorNoRepeat(const vector<T> &expected, const vector<T> &actual,
                               double tolerance = 1e-6)
{
    if (expected.size() != actual.size())
    {
        cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }
    size_t N = expected.size();
    unordered_set<T> s;
    for (size_t i = 0; i < expected.size(); i++)
        s.insert(expected.at(i));
    for (size_t i = 0; i < expected.size(); i++)
    {
        if (s.find(actual.at(i)) == s.end())
        {
            CoutError("Actual element " + to_string(actual.at(i)) + " is not found in expected vector");
        }
        else
        {
            s.erase(actual.at(i));
        }
    }

    return;
}

void AssertEigenEqualVector(Eigen::Matrix<double, Eigen::Dynamic, 1> &expected,
                            Eigen::Matrix<double, Eigen::Dynamic, 1> &actual)
{
    int m = expected.rows();
    int n = expected.cols();
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j));
        }
    }
}

void AssertEigenEqualMatrix(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &expected,
                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &actual)
{
    int m = expected.rows();
    int n = expected.cols();
    AssertEqualScalar(m, actual.rows());
    AssertEqualScalar(n, actual.cols());
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j));
        }
    }
}
