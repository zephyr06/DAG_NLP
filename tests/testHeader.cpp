#include <CppUnitLite/TestHarness.h>
#include "sources/Utils/testMy.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/OptimizeOrderUtils.h"
// #include "sources/Factors/RTDA_Factor.h"
// #include "sources/Factors/Interval.h"
// #include "sources/Optimization/TopologicalSort.h"
// #include "sources/Baseline/VerucchiRTDABridge.h"
// #include "sources/Utils/BatchUtils.h"
// #include "sources/batchOptimizeSFOrder.h"

#include <CppUnitLite/TestHarness.h>
#include "sources/TaskModel/DAG_Model.h"
#include "sources/Optimization/ScheduleSimulation.h"
#include "sources/Baseline/RTSS21IC.h"
#include "sources/Utils/profilier.h"
#include "sources/Utils/testMy.h"
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
