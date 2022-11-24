#include "sources/batchOptimizeSFOrder.h"

int main()
{
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM_Fast};
    OrderOptDAG_SPACE::BatchOptimizeOrder<OrderOptDAG_SPACE::OptimizeSF::RTSS21ICObj>(baselineMethods);
}