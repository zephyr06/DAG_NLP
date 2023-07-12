#include "sources/batchOptimizeSFOrder.h"

using namespace OrderOptDAG_SPACE;
int main() {
    std::vector<OrderOptDAG_SPACE::BASELINEMETHODS> baselineMethods = {
        OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM_Fast,
        OrderOptDAG_SPACE::TOM, OrderOptDAG_SPACE::TOM_IA};
    auto batch_results = OrderOptDAG_SPACE::BatchOptimizeOrder<
        OrderOptDAG_SPACE::OptimizeSF::ReactionTimeObj>(baselineMethods);
}