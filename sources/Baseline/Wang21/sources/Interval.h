#pragma once

#include "DeclareDAG.h"

namespace RTSS21IC_NLP
{

    struct Interval
    {
        double start;
        double length;
        LLint indexInSTV;

        Interval(double s1, double l1) : start(s1), length(l1) { indexInSTV = 0; }
        Interval(double s1, double l1, LLint i) : start(s1), length(l1), indexInSTV(i) {}
    };
    bool compare(Interval &i1, Interval &i2)
    {
        return (i1.start < i2.start);
    }
    /**
     * @brief always return v1 - v2;
     *
     * @param v1
     * @param v2
     * @return first double, overlap error
     */
    double Overlap(Interval &v1, Interval &v2)
    {
        double f1 = v1.start + v1.length;
        double f2 = v2.start + v2.length;
        if (v1.start >= f2 || v2.start >= f1)
            return 0;
        else if (v2.start <= v1.start && f2 >= v1.start && f1 >= f2)
        {
            return (f2 - v1.start);
        }
        else if (v2.start > v1.start && f2 < f1)
        {
            return v2.length;
        }
        else if (v1.start > v2.start && f1 < f2)
        {
            return v1.length;
        }
        else if (f1 >= v2.start && f2 >= f1 && v1.start <= v2.start)
        {
            return f1 - v2.start;
        }
        else
        {
            cout << Color::red << "Error in Overlap, no case found!" << Color::def << endl;
            throw;
        }
        return 0;
    }

    /**
     * @brief gradient w.r.t. start time of interval
     *
     * @param v1
     * @param v2
     * @return first double, gradient w.r.t. v1.start
     *         second double, gradient w.r.t. v2.start
     */
    pair<double, double> OverlapGradient(Interval v1, Interval v2)
    {
        // double e0=Overlap(v1,v2);
        v1.start += deltaOptimizer;
        double e_plus1 = Overlap(v1, v2);
        v1.start -= deltaOptimizer * 2;
        double e_minus1 = Overlap(v1, v2);
        v1.start += deltaOptimizer;

        v2.start += deltaOptimizer;
        double e_plus2 = Overlap(v1, v2);
        v2.start -= deltaOptimizer * 2;
        double e_minus2 = Overlap(v1, v2);

        return make_pair((e_plus1 - e_minus1) / 2 / deltaOptimizer, (e_plus2 - e_minus2) / 2 / deltaOptimizer);
        // double f1 = v1.start + v1.length;
        // double f2 = v2.start + v2.length;
        // if (v1.start >= f2 || v2.start >= f1)
        //     return make_pair(0, 0);
        // else if (v2.start <= v1.start && f2 >= v1.start && f1 >= f2)
        // {
        //     if (v2.start == v1.start)
        //     {
        //         if (f1 == f2)
        //         {
        //             return make_pair(0, 0);
        //         }
        //         else
        //         {
        //             return make_pair(-0.5, 0.5);
        //         }
        //     }
        //     else
        //     {
        //         if(f1==f2)
        //         {

        //         }
        //     }

        //     return make_pair(-1, 1);
        // }
        // else if (v2.start > v1.start && f2 < f1)
        // {
        //     return make_pair(0, 0);
        // }
        // else if (v1.start > v2.start && f1 < f2)
        // {
        //     return make_pair(0, 0);
        // }
        // else if (f1 >= v2.start && f2 >= f1 && v1.start <= v2.start)
        // {
        //     return make_pair(1, -1);
        // }
        // else
        // {
        //     cout << Color::red << "Error in Overlap, no case found!" << Color::def << endl;
        //     throw;
        // }
        // return make_pair(0, 0);
    }

    double IntervalOverlapError(vector<Interval> &intervalVec)
    {
        sort(intervalVec.begin(), intervalVec.end(), compare);

        double overlapAll = 0;
        int n = intervalVec.size();
        for (int i = 0; i < n; i++)
        {
            double endTime = intervalVec[i].start + intervalVec[i].length;
            for (int j = i + 1; j < n; j++)
            {
                if (intervalVec[j].start >= endTime)
                    break;
                else
                {
                    double ttttt = Overlap(intervalVec[i], intervalVec[j]);
                    overlapAll += ttttt;
                }
            }
        }
        return overlapAll;
    }
} // namespace RTSS21IC_NLP
