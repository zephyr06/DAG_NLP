#include "sources/BatchOptimization/batchOptimizeSFOrder.h"

int main() {
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {
        OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM};
    OrderOptDAG_SPACE::BatchOptimizeOrder<
        OrderOptDAG_SPACE::OptimizeSF::SensorFusionObj>(baselineMethods);
}