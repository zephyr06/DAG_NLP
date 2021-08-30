#include "../sources/GenerateRandomTaskset.h"
#include "../sources/argparse.hpp"

int main(int argc, char *argv[])
{
    argparse::ArgumentParser program("program name");
    program.add_argument("-v", "--verbose"); // parameter packing

    program.add_argument("--N")
        .default_value(5)
        .help("the number of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--taskSetNumber")
        .default_value(10)
        .help("the number DAGs to create")
        .scan<'i', int>();
    program.add_argument("--totalUtilization")
        .default_value(0.4)
        .help("the total utilization of tasks in each DAG")
        .scan<'f', double>();
    program.add_argument("--NumberOfProcessor")
        .default_value(2)
        .help("the NumberOfProcessor of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--periodMin")
        .default_value(100)
        .help("the minimum period of tasks in DAG")
        .scan<'i', int>();
    program.add_argument("--periodMax")
        .default_value(1000)
        .help("the maximum period of tasks in DAG")
        .scan<'i', int>();

    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error &err)
    {
        std::cout << err.what() << std::endl;
        std::cout << program;
        exit(0);
    }

    int N = program.get<int>("--N");
    int DAG_Number = program.get<int>("--taskSetNumber");
    double totalUtilization = program.get<double>("--totalUtilization");
    int NumberOfProcessor = program.get<int>("--NumberOfProcessor");
    int periodMin = program.get<int>("--periodMin");
    int periodMax = program.get<int>("--periodMax");
    cout << N << ", " << DAG_Number << ", " << totalUtilization << ", " << NumberOfProcessor << ", " << periodMin << ", " << periodMax << ", " << endl;

    return 0;
}