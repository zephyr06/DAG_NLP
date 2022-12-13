#include "sources/batchOptimizeSFOrder.h"
// #include "sources/Utils/profilier.h"
int main()
{
    // BeginTimer("main");
    std::vector<OrderOptDAG_SPACE::BaselineMethods> baselineMethods = {OrderOptDAG_SPACE::InitialMethod, OrderOptDAG_SPACE::TOM_FastLP};
    OrderOptDAG_SPACE::BatchOptimizeOrder<OrderOptDAG_SPACE::OptimizeSF::RTDAExperimentObj>(baselineMethods);
//     EndTimer("main");
//     PrintTimer();
}