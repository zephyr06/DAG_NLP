#include "sources/Factors/Interval.h"

double Overlap(Interval &v1, Interval &v2)
{
    double f1 = v1.start + v1.length;
    double f2 = v2.start + v2.length;
    int coreRequireDiff = 1 - v1.coreRequire - v2.coreRequire;
    int coreError = 0;
    if (coreRequireDiff >= 0)
        return 0;
    else
        coreError = -1 * coreRequireDiff;

    if (v1.start >= f2 || v2.start >= f1)
        return 0;
    else if (v2.start <= v1.start && f2 >= v1.start && f1 >= f2)
    {
        return (f2 - v1.start) * coreError;
    }
    else if (v2.start > v1.start && f2 < f1)
    {
        return v2.length * coreError;
        // return coreError * (v1.length + v2.length - (f2 - v1.start));
    }
    else if (v1.start > v2.start && f1 < f2)
    {
        return v1.length * coreError;
        // return coreError * (v2.length + v1.length - (f2 - v1.start));
    }
    else if (f1 >= v2.start && f2 >= f1 && v1.start <= v2.start)
    {
        return (f1 - v2.start) * coreError;
    }
    else
    {
        std::cout << Color::red << "Error in Overlap, no case found!" << Color::def << std::endl;
        throw;
    }
    return 0;
}

std::pair<double, double> OverlapGradient(Interval v1, Interval v2)
{
    // double e0=Overlap(v1,v2);
    v1.start += GlobalVariablesDAGOpt::deltaOptimizer;
    double e_plus1 = Overlap(v1, v2);
    v1.start -= GlobalVariablesDAGOpt::deltaOptimizer * 2;
    double e_minus1 = Overlap(v1, v2);
    v1.start += GlobalVariablesDAGOpt::deltaOptimizer;

    v2.start += GlobalVariablesDAGOpt::deltaOptimizer;
    double e_plus2 = Overlap(v1, v2);
    v2.start -= GlobalVariablesDAGOpt::deltaOptimizer * 2;
    double e_minus2 = Overlap(v1, v2);

    return std::make_pair((e_plus1 - e_minus1) / 2 / GlobalVariablesDAGOpt::deltaOptimizer, (e_plus2 - e_minus2) / 2 / GlobalVariablesDAGOpt::deltaOptimizer);
    // double f1 = v1.start + v1.length;
    // double f2 = v2.start + v2.length;
    // if (v1.start >= f2 || v2.start >= f1)
    //     return std::make_pair(0, 0);
    // else if (v2.start <= v1.start && f2 >= v1.start && f1 >= f2)
    // {
    //     if (v2.start == v1.start)
    //     {
    //         if (f1 == f2)
    //         {
    //             return std::make_pair(0, 0);
    //         }
    //         else
    //         {
    //             return std::make_pair(-0.5, 0.5);
    //         }
    //     }
    //     else
    //     {
    //         if(f1==f2)
    //         {

    //         }
    //     }

    //     return std::make_pair(-1, 1);
    // }
    // else if (v2.start > v1.start && f2 < f1)
    // {
    //     return std::make_pair(0, 0);
    // }
    // else if (v1.start > v2.start && f1 < f2)
    // {
    //     return std::make_pair(0, 0);
    // }
    // else if (f1 >= v2.start && f2 >= f1 && v1.start <= v2.start)
    // {
    //     return std::make_pair(1, -1);
    // }
    // else
    // {
    //    std::cout << Color::red << "Error in Overlap, no case found!" << Color::def <<std::endl;
    //     throw;
    // }
    // return std::make_pair(0, 0);
}

double IntervalOverlapError(std::vector<Interval> &intervalVec)
{
    if (intervalVec.size() <= 1)
        return 0;
    sort(intervalVec.begin(), intervalVec.end(), compare);

    double overlapAll = 0;
    size_t n = intervalVec.size();
    for (size_t i = 0; i < n; i++)
    {
        double endTime = intervalVec[i].start + intervalVec[i].length;
        for (size_t j = i + 1; j < n; j++)
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