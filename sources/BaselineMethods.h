#pragma once
#include <string>
#include <vector>

namespace OrderOptDAG_SPACE {
// !!!if adding more methods, need to update BaselineMethodNames below !!!
// Note: *_Approx should not be considered as an independent method?

// if adding more methods, need to update WriteScheduleToFile() and
// GlobalVariablesDAGOpt::TotalMethodUnderComparison
enum BASELINEMETHODS {
    InitialMethod,  // 0
    Verucchi20,     // 1
    Wang21,         // 2
    TOM,            // 3
    TOM_Fast,       // 4
    GlobalOpt       // 5
};
const std::vector<std::string> BaselineMethodNames = {
    "InitialMethod", "Verucchi20", "Wang21", "TOM", "TOM_Fast", "GlobalOpt"};

}  // namespace OrderOptDAG_SPACE