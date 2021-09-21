#include <iostream>
#include <chrono>
#include <cmath>
#include <time.h>
#include "Optimize.h"
#include "../includeMoe/moe/moe.hpp"

using namespace DAG_SPACE;

VectorDynamic OptimizeSchedulingSA(DAG_Model &dagTasks)
{
    TaskSet tasks = dagTasks.tasks;
    int N = tasks.size();
    LLint hyperPeriod = HyperPeriod(tasks);

    // declare variables
    vector<LLint> sizeOfVariables;
    int variableDimension = 0;
    for (int i = 0; i < N; i++)
    {
        LLint size = hyperPeriod / tasks[i].period;
        sizeOfVariables.push_back(size);
        variableDimension += size;
    }
    moe::SimulatedAnnealing<double> moether(moe::SAParameters<double>()
                                                .withTemperature(temperatureSA)
                                                .withCoolingRate(coolingRateSA)
                                                .withDimensions(variableDimension + 1)
                                                .withRange({0, double(hyperPeriod)}));

    moether.setFitnessFunction([&](auto startTimeVec) -> double
                               {
                                   VectorDynamic startTimeVector = Vector2Eigen<double>(startTimeVec.genotype);

                                   return GraphErrorEvaluation(dagTasks, startTimeVector) * -1;
                               });

    auto start = std::chrono::high_resolution_clock::now();
    VectorDynamic initialEstimate = GenerateInitialForDAG(dagTasks, sizeOfVariables, variableDimension);
    cout << "Initial estimation for SA is " << initialEstimate << endl;
    auto initialSA = Eigen2Vector<double>(initialEstimate);
    moether.runSA(SA_iteration, initialSA, randomInitialize);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = end - start;

    auto best_moe = moether.getBestMoe();

    std::cout
        << "fitness: " << best_moe.fitness * -1 << "\n"
        << "time spent: " << diff.count() << " seconds" << std::endl;
    return Vector2Eigen<double>(best_moe.genotype);
}
