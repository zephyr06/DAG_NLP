#include "sources/BatchOptimization/batchOptimizeSFOrder.h"
#include "sources/Utils/argparse.hpp"

using namespace OrderOptDAG_SPACE;
int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("BatchSF_TOM");
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
    OrderOptDAG_SPACE::BatchSettings batch_test_settings(
        N, begin_index, end_index, "TaskData/N" + std::to_string(N) + "/");
    
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {
        OrderOptDAG_SPACE::InitialMethod, 
        OrderOptDAG_SPACE::TOM_Fast,
        // OrderOptDAG_SPACE::TOM_IA,
        OrderOptDAG_SPACE::TOM,
        OrderOptDAG_SPACE::TOM_Raw,
        OrderOptDAG_SPACE::TOM_FarReach};
    OrderOptDAG_SPACE::BatchOptimizeOrder<
        OrderOptDAG_SPACE::OptimizeSF::SensorFusionObj>(baselineMethods, batch_test_settings);
}