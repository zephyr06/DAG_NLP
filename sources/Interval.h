#include "DeclareDAG.h"

struct Interval
{
    double start;
    double length;
};
bool compare(Interval &i1, Interval &i2)
{
    return (i1.start < i2.start);
}
inline double Overlap(Interval &v1, Interval &v2)
{
    return v1.start + v1.length - v2.start;
}