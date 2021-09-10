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
    string path = "/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v8.csv";
    DAG_Model dm = ReadDAG_Tasks(path);
    dm.print();
    AssertBool(true, dm.mapPrev[3].size() == 3);
    AssertBool(true, dm.mapPrev[4][0].id == 3);
}

TEST(ReadDAG_Tasks, v2)
{
    string path = "/home/zephyr/Programming/DAG_NLP/TaskData/test_n5_v15.csv";
    DAG_Model dm = ReadDAG_Tasks(path);
    dm.print();
    AssertBool(true, dm.mapPrev[1].size() == 2);
    AssertBool(true, dm.mapPrev[2][0].id == 4);
    AssertBool(true, dm.mapPrev[0][0].id == 4);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}