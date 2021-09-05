#include <iostream>
#include <chrono>
#include <cmath>
#include <time.h>
#include "Optimize.h"
#include "../includeMoe/moe/moe.hpp"

using namespace DAG_SPACE;

vector<double> Eigen2Vector(VectorDynamic &input)
{
    vector<double> res;
    LLint len = input.rows();
    res.reserve(len);
    for (LLint i = 0; i < len; i++)
        res.push_back(input(i, 0));
    return res;
}
VectorDynamic Vector2Eigen(vector<double> &input)
{

    LLint len = input.size();
    VectorDynamic res;
    res.resize(len, 1);
    for (LLint i = 0; i < len; i++)
        res(i, 0) = input[i];
    return res;
}

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

    MAP_Index2Data mapIndex;
    for (LLint i = 0; i < variableDimension; i++)
    {
        MappingDataStruct m{i, 0};
        mapIndex[i] = m;
    }
    bool whetherEliminate = false;
    vector<bool> maskForEliminate(variableDimension, false);

    moe::SimulatedAnnealing<double> moether(moe::SAParameters<double>()
                                                .withTemperature(temperatureSA)
                                                .withCoolingRate(coolingRateSA)
                                                .withDimensions(variableDimension)
                                                .withRange({0, double(hyperPeriod)}));

    moether.setFitnessFunction([&](auto startTimeVec) -> double
                               {
                                   VectorDynamic startTimeVector = Vector2Eigen(startTimeVec.genotype);

                                   // build the factor graph
                                   NonlinearFactorGraph graph;
                                   Symbol key('a', 0);

                                   LLint errorDimensionDAG = 1 + 4;
                                   auto model = noiseModel::Isotropic::Sigma(errorDimensionDAG, noiseModelSigma);
                                   graph.emplace_shared<DAG_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                                              errorDimensionDAG, mapIndex,
                                                                              maskForEliminate, model);
                                   LLint errorDimensionDBF = 1;
                                   model = noiseModel::Isotropic::Sigma(errorDimensionDBF, noiseModelSigma);
                                   graph.emplace_shared<DBF_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                                              errorDimensionDBF, mapIndex,
                                                                              maskForEliminate,
                                                                              model);
                                   LLint errorDimensionDDL = 2 * variableDimension;
                                   model = noiseModel::Isotropic::Sigma(errorDimensionDDL, noiseModelSigma);
                                   graph.emplace_shared<DDL_ConstraintFactor>(key, dagTasks.tasks, sizeOfVariables,
                                                                              errorDimensionDDL, mapIndex,
                                                                              maskForEliminate, model);
                                   LLint errorDimensionSF = sizeOfVariables[3];
                                   model = noiseModel::Isotropic::Sigma(errorDimensionSF, noiseModelSigma);
                                   graph.emplace_shared<SensorFusion_ConstraintFactor>(key, dagTasks, sizeOfVariables,
                                                                                       errorDimensionSF, sensorFusionTolerance,
                                                                                       mapIndex, maskForEliminate, model);

                                   Values initialEstimateFG;
                                   initialEstimateFG.insert(key, startTimeVector);
                                   return graph.error(initialEstimateFG);
                               });

    auto start = std::chrono::high_resolution_clock::now();
    VectorDynamic initialEstimate = GenerateInitialForDAG(tasks, sizeOfVariables, variableDimension);
    auto initialSA = Eigen2Vector(initialEstimate);
    moether.runSA(SA_iteration, initialSA, randomInitialize);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = end - start;

    auto best_moe = moether.getBestMoe();

    std::cout << "genotype: " << best_moe.genotype[0] << "\t" << best_moe.genotype[1] << "\n"
              << "fitness: " << best_moe.fitness << "\n"
              << "time spent: " << diff.count() << " seconds" << std::endl;
    return Vector2Eigen(best_moe.genotype);
}
