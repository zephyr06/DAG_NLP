#include "DeclareDAG.h"

struct Interval
{
    double start;
    double length;

    Interval(double s1, double l1) : start(s1), length(l1) {}
};
bool compare(Interval &i1, Interval &i2)
{
    return (i1.start < i2.start);
}
double Overlap(Interval &v1, Interval &v2)
{
    double f1 = v1.start + v1.length;
    double f2 = v2.start + v2.length;
    if (v1.start >= f2 || v2.start >= f1)
        return 0;
    else if (v2.start <= v1.start && f2 >= v1.start && f1 >= f2)
    {
        return f2 - v1.start;
    }
    else if (v2.start >= v1.start && f2 <= f1)
    {
        return v2.length;
    }
    else if (v1.start >= v2.start && f1 <= f2)
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

double IntervalOverlapError(vector<Interval> &intervalVec)
{
    sort(intervalVec.begin(), intervalVec.end(), compare);

    double overlapAll = 0;
    int n = intervalVec.size();
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