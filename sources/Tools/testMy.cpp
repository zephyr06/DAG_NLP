#include "testMy.h"

void AssertEqualScalar(double expected, double actual, double tolerance, int lineNumber)
{
    if (expected != 0)
    {
        if (abs((expected - actual) / expected) < tolerance)
            return;
        else
        {
            if (lineNumber != 0)
                std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
            AssertUnEqual<double>(expected, actual);
        }
    }
    else
    {
        if (actual != 0)
        {
            if (lineNumber != 0)
                std::cout << Color::red << "Line Number: " << std::to_string(lineNumber) << Color::def << std::endl;
            AssertUnEqual<double>(expected, actual);
        }
    }
}

template <typename T>
void AssertEqualVectorNoRepeat(const std::vector<T> &expected, const std::vector<T> &actual,
                               double tolerance, int lineNumber)
{
    if (expected.size() != actual.size())
    {
        std::cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }
    // size_t N = expected.size();
    std::unordered_set<T> s;
    for (size_t i = 0; i < expected.size(); i++)
        s.insert(expected.at(i));
    for (size_t i = 0; i < expected.size(); i++)
    {
        if (s.find(actual.at(i)) == s.end())
        {
            CoutError("Actual element " + std::to_string(actual.at(i)) + " is not found in expected vector");
        }
        else
        {
            s.erase(actual.at(i));
        }
    }

    return;
}

template <typename T>
void AssertEqualVectorExact(const std::vector<T> &expected, const std::vector<T> &actual,
                            double tolerance, int lineNumber)
{
    if (expected.size() != actual.size())
    {
        std::cout << Color::red << "Length error! " << Color::def;
        AssertUnEqual(expected.size(), actual.size());
        return;
    }

    for (size_t i = 0; i < expected.size(); i++)
    {
        if (expected[i] != actual[i])
        {
            CoutError("Expected element at " + std::to_string(i) +
                      " does not match actual element at " + std::to_string((i)));
        }
    }

    return;
}

void AssertEigenEqualVector(Eigen::Matrix<double, Eigen::Dynamic, 1> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, 1> actual, int lineNumber)
{
    int m = expected.rows();
    int n = expected.cols();
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j), 1e-6, lineNumber);
        }
    }
}

void AssertEigenEqualMatrix(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> expected,
                            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> actual, int lineNumber)
{
    int m = expected.rows();
    int n = expected.cols();
    AssertEqualScalar(m, actual.rows(), 1e-6, lineNumber);
    AssertEqualScalar(n, actual.cols(), 1e-6, lineNumber);
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < n; j++)
        {
            AssertEqualScalar(expected(i, j), actual(i, j), 1e-6, lineNumber);
        }
    }
}

template <class T1, class T2>
void AssertEqualMap(std::unordered_map<T1, T2> &mExpect, std::unordered_map<T1, T2> &mActual)
{
    using namespace std;
    if (mExpect.size() != mActual.size())
    {
        CoutWarning("Size error!");
        AssertEqualScalar(mExpect.size(), mActual.size());
    }
    for (auto itr = mExpect.begin(); itr != mExpect.end(); itr++)
    {
        auto itrActual = mActual.find(itr->first);
        if (itrActual == mActual.end() || (itrActual->second).notEqual(itr->second))
        {

            try
            {
                std::cout << "Expect is " << itr->first << ", " << itr->second << endl;
                std::cout << "Actual is " << itrActual->first << ", " << itrActual->second << endl;
            }
            catch (const std::exception &e)
            {
                std::cout << "Cannot print the key to show mismatch element\n";
            }
            CoutError("Element in mExpect is not found in or not equal to mActual!");
        }
    }
}

double SquareError(std::vector<double> &seq)
{
    double res = 0;
    for (double x : seq)
    {
        res += x * x / 2.0;
    }
    return res;
}