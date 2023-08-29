#include "sources/Optimization/OptimizeSFOrder.h"
#include "sources/Utils/argparse.hpp"
using namespace GlobalVariablesDAGOpt;

int main(int argc, char *argv[])
{
    argparse::ArgumentParser program("program name");
    program.add_argument("-v", "--verbose"); // parameter packing

    program.add_argument("--N")
        .default_value(5)
        .help("the folder of task sets to run experiments")
        .scan<'i', int>();
    program.add_argument("--begin")
        .default_value(0)
        .help("the first file index that's going to be optimized")
        .scan<'i', int>();
    program.add_argument("--end")
        .default_value(1000)
        .help(
            "the last file index that's going to be optimized (Not INCLUSIVE)")
        .scan<'i', int>();

    // program.add_argument("--obj")
    //     .default_value(std::string("DA"))
    //     .help("the type of objective function,  DA");

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
    int begin_index = program.get<int>("--begin");
    int end_index = program.get<int>("--end");
    // std::string obj_type = program.get<std::string>("--obj");

    BeginTimer("main");
    using namespace OrderOptDAG_SPACE;
    int chainNum = GlobalVariablesDAGOpt::NumCauseEffectChain;
    for (int file_index = begin_index; file_index < end_index; file_index++)
    {
        DAG_Model dagTasks =
            ReadDAG_Tasks(GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/N" + std::to_string(N) + "/" +
                              "dag-set-N" + std::to_string(N) + "-" +
                              std::string(3 - std::to_string(file_index).size(), '0') +
                              std::to_string(file_index) + "-syntheticJobs" + ".csv",
                          "orig", chainNum);

        ScheduleResult sth;
        OrderOptDAG_SPACE::OptimizeSF::ScheduleOptions scheduleOption;
        scheduleOption.LoadParametersYaml();
        // LPOrderScheduler
        sth = OrderOptDAG_SPACE::OptimizeSF::ScheduleDAGModel<
            LPOrderScheduler, OrderOptDAG_SPACE::OptimizeSF::DataAgeObj>(
            dagTasks, scheduleOption);
        PrintResultAnalyzation(sth, dagTasks);
        std::cout << GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/N" + std::to_string(N) + "/" +
                         "dag-set-N" + std::to_string(N) + "-" +
                         std::string(3 - std::to_string(file_index).size(), '0') +
                         std::to_string(file_index) + "-syntheticJobs" + ".csv\n";
        std::cout << "Schedulable? " << sth.schedulable_ << std::endl;
    }
    EndTimer("main");
    PrintTimer();
}