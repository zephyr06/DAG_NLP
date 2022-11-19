
#include "sources/batchOptimizeSFOrder.h"

int main()
{
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::Verucchi20};
    OrderOptDAG_SPACE::BatchOptimizeOrder(baselineMethods);
}