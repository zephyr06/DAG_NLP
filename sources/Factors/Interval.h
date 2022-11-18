#pragma once
#include "sources/Utils/DeclareDAG.h"
#include "sources/Utils/colormod.h"

struct Interval
{
    double start;
    double length;
    int coreRequire;
    LLint indexInSTV;

    Interval(double s1, double l1) : start(s1), length(l1)
    {
        indexInSTV = 0;
        coreRequire = 1;
    }
    Interval(double s1, double l1, LLint i) : start(s1),
                                              length(l1),
                                              indexInSTV(i) { coreRequire = 1; }
    Interval(double s1, double l1, LLint i, int coreRequire) : start(s1),
                                                               length(l1),
                                                               coreRequire(coreRequire),
                                                               indexInSTV(i) {}

    bool IfBelong(double x)
    {
        return x >= start && x <= start + length;
    }
    // whether interval x belongs to this
    bool IfBelong(Interval x)
    {
        return start <= x.start && x.start + x.length <= start + length;
    }
    void UpdateSF(double startI, double finish)
    {
        start = startI;
        length = finish - start;
    }
};
inline bool compare(Interval &i1, Interval &i2)
{
    return (i1.start < i2.start);
}

/**
 * @brief always return v1 - v2;
 *
 *
 * @param v1
 * @param v2
 * @return double, overlap error
 */
double Overlap(Interval &v1, Interval &v2);
/**
 * @brief gradient w.r.t. start time of interval
 *
 * @param v1
 * @param v2
 * @return first double, gradient w.r.t. v1.start
 *         second double, gradient w.r.t. v2.start
 */
std::pair<double, double> OverlapGradient(Interval v1, Interval v2);

double IntervalOverlapError(std::vector<Interval> &intervalVec);