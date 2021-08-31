#include "../sources/GenerateRandomTaskset.h"
#include "../sources/testMy.h"

TEST(ExtractVariable, v1)
{
    int N = 5;
    double sumUtil = 0.6;
    auto sth = Uunifast(N, sumUtil);
    double all = 0;
    for (int i = 0; i < N; i++)
        all += sth[i];
    AssertEqualScalar(sumUtil, all);
}

TEST(ReadDAG_Tasks, V1)
{
    string path = "/home/zephyr/Programming/DAG_NLP/TaskData/dat-test-n5-v7.csv";
    auto dm = ReadDAG_Tasks(path);
    dm.print();
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}