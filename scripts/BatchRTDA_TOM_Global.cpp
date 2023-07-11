#include "sources/batchOptimizeSFOrder.h"

int main() {
  std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {
      OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM, OrderOptDAG_SPACE::GlobalOpt};
  std::string dataSetFolder = GlobalVariablesDAGOpt::PROJECT_PATH + "TaskData/CompareAgainstGlobal/";
  OrderOptDAG_SPACE::BatchOptimizeOrder<OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(baselineMethods,
                                                                                          dataSetFolder);
}