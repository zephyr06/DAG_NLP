#pragma once
#include <string>
#include <vector>

namespace OrderOptDAG_SPACE {
// !!!if adding more methods, need to update BaselineMethodNames below !!!
// Note: *_Approx should not be considered as an independent method?

// if adding more methods, need to update WriteScheduleToFile() and
// GlobalVariablesDAGOpt::TotalMethodUnderComparison
enum BASELINEMETHODS {
    InitialMethod,
    Verucchi20,
    Wang21,
    SimulatedAnnealing,
    TOM,
    TOM_Fast,
    TOM_IA,
    TOM_Threshold,
    TOM_RandomSearch,
    TOM_Raw,
    TOM_FarReach,
    GlobalOpt
};
const std::vector<std::string> BaselineMethodNames = {
    "InitialMethod", "Verucchi20", "Wang21",  "SimulatedAnnealing", "TOM",
    "TOM_Fast", "TOM_IA", "TOM_Threshold", "TOM_RandomSearch",
    "TOM_Raw", "TOM_FarReach", "GlobalOpt"};

}  // namespace OrderOptDAG_SPACE