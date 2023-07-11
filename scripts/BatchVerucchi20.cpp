#include "sources/batchOptimizeSFOrder.h"

int main()
{
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::Verucchi20};
    OrderOptDAG_SPACE::BatchOptimizeOrder<OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(baselineMethods);
}