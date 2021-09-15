#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "colormod.h"

using namespace std;

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
void AssertEqualVector(T expected, T actual, double tolerance = 1e-6)
{
    if (expected.size() != actual.size())
    {
        cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }
    size_t N = expected.size();
    for (size_t i = 0; i < N; i++)
    {
        AssertEqualScalar(expected[i], actual[i]);
    }
    return;
}

void CoutError(string message)
{
    cout << Color::red << message << Color::def << endl;
    throw;
}