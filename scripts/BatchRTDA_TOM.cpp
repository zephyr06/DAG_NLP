#include "sources/batchOptimizeSFOrder.h"

int main()
{
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM};
    OrderOptDAG_SPACE::BatchOptimizeOrder<OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(baselineMethods);
}