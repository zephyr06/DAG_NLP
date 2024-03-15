#pragma once

#include "includeMoe/moe/moe.hpp"
#include "sources/Factors/ObjectiveFunctions.h"
#include "sources/Optimization/InitialEstimate.h"
#include "sources/Optimization/ScheduleOptions.h"
#include "sources/Utils/MatirxConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/Utils/ScheduleResults.h"
#include <chrono>
#include <cmath>
#include <iostream>
#include <time.h>

namespace OrderOptDAG_SPACE {

std::vector<std::pair<int, int>> GetJobMinMaxStartTimeRange(const TaskSetInfoDerived &tasks_info);

// NOTE: time limit is not used in this function!!!!
template <typename ObjectiveFunctionBase>
ScheduleResult OptimizeSchedulingSA(OrderOptDAG_SPACE::DAG_Model &dagTasks,
                                    const OptimizeSF::ScheduleOptions &scheduleOptions,
                                    double timeLimits = GlobalVariablesDAGOpt::OPTIMIZE_TIME_LIMIT) {
  TaskSet tasks = dagTasks.tasks;
  RegularTaskSystem::TaskSetInfoDerived tasks_info(tasks);

  std::vector<uint> processorJobVec;
  VectorDynamic initialEstimate =
      ListSchedulingLFTPA(dagTasks, tasks_info, scheduleOptions.processorNum_, processorJobVec);

  moe::SimulatedAnnealing<double> moether(moe::SAParameters<double>()
                                              .withTemperature(GlobalVariablesDAGOpt::temperatureSA)
                                              .withCoolingRate(GlobalVariablesDAGOpt::coolingRateSA)
                                              .withDimensions(tasks_info.variableDimension + 1)
                                              .withRange({0, double(tasks_info.hyper_period)}));
  moether.AddJobMinMaxStartTimeRange(GetJobMinMaxStartTimeRange(tasks_info));

  moether.setFitnessFunction([&](auto startTimeVec) -> double {
    VectorDynamic startTimeVector = Vector2Eigen<double>(startTimeVec.genotype);
    // VectorDynamic startTimeVector;
    double obj_main =
        ObjectiveFunctionBase::TrueObj(dagTasks, tasks_info, startTimeVector, scheduleOptions) * -1;
    bool schedulable = ExamAll_Feasibility(dagTasks, tasks_info, startTimeVector, processorJobVec,
                                           scheduleOptions.processorNum_);
    std::cout << "Obj during SA: " << obj_main << "\n";
    if (schedulable)
      return obj_main;
    else
      return obj_main - 1e9;
  });

  auto start = std::chrono::high_resolution_clock::now();

  if (GlobalVariablesDAGOpt::debugMode)
    std::cout << "Initial estimation for SA is " << initialEstimate << std::endl;
  auto initialSA = Eigen2Vector<double>(initialEstimate);
  moether.runSA(GlobalVariablesDAGOpt::SA_iteration, initialSA, GlobalVariablesDAGOpt::randomInitialize,
                timeLimits);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> diff = end - start;

  auto best_moe = moether.getBestMoe();
  if (GlobalVariablesDAGOpt::debugMode == 1)
    std::cout << "fitness after optimization: " << best_moe.fitness * -1 << "\n"
              << "time spent: " << diff.count() << " seconds" << std::endl;
  bool schedulable = true;
  if (best_moe.fitness * -1 > 1e9) {
    schedulable = false;
  }

  VectorDynamic start_time_vector_result = Vector2Eigen<double>(best_moe.genotype);
  return ScheduleResult{SFOrder(), start_time_vector_result, schedulable, best_moe.fitness * -1};
}

} // namespace OrderOptDAG_SPACE
