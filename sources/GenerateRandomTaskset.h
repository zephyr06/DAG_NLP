#include <time.h>
#include "RegularTasks.h"
#include <boost/program_options/options_description.hpp>
using namespace RegularTaskSystem;
namespace po = boost::program_options;

vector<double> Uunifast(int N, double utilAll)
{
    double sumU = utilAll;
    vector<double> utilVec(N, 0);
    srand(time(0));
    double nextU;
    for (size_t i = 1; i < N; i++)
    {
        nextU = sumU * pow(double(rand()) / RAND_MAX, 1.0 / (N - 1));
        utilVec[i - 1] = sumU - nextU;
        sumU = nextU;
    }
    utilVec[N - 1] = nextU;
    return utilVec;
}
