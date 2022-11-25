/*
 * Evaluation.cpp
 *
 *  Created on: May 6, 2019
 *      Author: mirco
 */
#include <sources/Baseline/Verucchi20/Evaluation/Evaluation.h>
#include <sources/Baseline/Verucchi20/Evaluation/LatencyInfo.h>
#include <sources/Baseline/Verucchi20/Evaluation/Scheduling.h>
#include <sources/Baseline/Verucchi20/MultiRate/MultiRateTaskset.h>
#include <cmath>
#include <iostream>
#include "sources/Baseline/VerucchiRTDABridge.h"

void Evaluation::addLatency(const Chain &chain, const LatencyCost &cost,
							const LatencyConstraint &constraint)
{
	latencyEval_.push_back(std::make_pair(chain, std::make_pair(cost, constraint)));
}

// schedule each DAG task set separately, and return a cost which evaluates which dag is the best
const DAG &
Evaluation::evaluate(const std::vector<DAG> &dags)
{
	std::vector<float> cost(dags.size(), 0.0);

	unsigned invalidDags = 0;
	for (const auto &eval : latencyEval_)
	{
		std::vector<unsigned> chain = taskChainToNum(eval.first);

		for (unsigned k = 0; k < dags.size(); k++)
		{
			if (std::isnan(cost[k]))
				continue;

			auto info = dags[k].getLatencyInfo(chain);

			if (!eval.second.second.isValid(info))
			{
				cost[k] = NAN;
				invalidDags++;
				continue;
			}

			cost[k] += eval.second.first.getCost(info);
		}
	}

	for (unsigned k = 0; k < dags.size(); k++)
	{
		if (std::isnan(cost[k]))
			continue;

		SchedulingInfo info = getSchedulingInfo(dags[k], schedulingEval_.second);

		if (!schedulingEval_.second.isValid(info))
		{
			cost[k] = NAN;
			invalidDags++;
			continue;
		}

		cost[k] += schedulingEval_.first.getCost(info);
	}

	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Num invalid dags: " << invalidDags << std::endl;
	}
	if (invalidDags == dags.size())
	{
		std::cout << "No valid dag found. Constraints are too tight." << std::endl;
		return dags[0];
	}

	unsigned bestDAG = 0;
	float minCost = std::numeric_limits<float>::max();
	for (unsigned k = 0; k < cost.size(); k++)
	{
		if (!std::isnan(cost[k]) && cost[k] < minCost)
		{
			bestDAG = k;
			minCost = cost[k];
		}
	}

	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Best DAG: " << bestDAG << ", with total cost: " << minCost << std::endl
				  << std::endl;
		for (const auto &eval : latencyEval_)
		{
			printChain(eval.first);
			std::cout << dags[bestDAG].getLatencyInfo(taskChainToNum(eval.first)) << std::endl;
		}
	}

	return dags[bestDAG];
}

// schedule each DAG task set separately using RTDA, and return a cost which evaluates which dag is the best
const DAG &
Evaluation::evaluateWithRTDA(const std::vector<DAG> &dags)
{
	std::vector<float> cost(dags.size(), 0.0);

	unsigned invalidDags = 0;

	for (unsigned k = 0; k < dags.size(); k++)
	{
		if (std::isnan(cost[k]))
			continue;

		SchedulingInfo info = getSchedulingInfo(dags[k], schedulingEval_.second);

		if (!schedulingEval_.second.isValid(info))
		{
			cost[k] = NAN;
			invalidDags++;
			continue;
		}

		cost[k] += schedulingEval_.first.getCost(info);
	}

	for (const auto &eval : latencyEval_)
	{
		std::vector<unsigned> chain = taskChainToNum(eval.first);

		for (unsigned k = 0; k < dags.size(); k++)
		{
			if (std::isnan(cost[k]))
				continue;

			auto info = getLatencyInfoRTDA(dags[k], chain);

			if (!eval.second.second.isValid(info))
			{
				cost[k] = NAN;
				invalidDags++;
				continue;
			}

			cost[k] += eval.second.first.getCost(info);
		}
	}
	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Num invalid dags: " << invalidDags << std::endl;
	}

	if (invalidDags == dags.size())
	{
		if (GlobalVariablesDAGOpt::debugMode)
		{
			std::cout << "No valid dag found. Constraints are too tight." << std::endl;
		}
		std::shared_ptr<DAG> empty_dag = std::shared_ptr<DAG>(new DAG(-1));
		return *empty_dag;
	}

	unsigned bestDAG = 0;
	float minCost = std::numeric_limits<float>::max();
	for (unsigned k = 0; k < cost.size(); k++)
	{
		if (!std::isnan(cost[k]) && cost[k] < minCost)
		{
			bestDAG = k;
			minCost = cost[k];
		}
	}

	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Best DAG: " << bestDAG << ", with total cost: " << minCost << std::endl
				  << std::endl;
		for (const auto &eval : latencyEval_)
		{
			printChain(eval.first);
			auto info = getLatencyInfoRTDA(dags[bestDAG], taskChainToNum(eval.first));
			std::cout << info << std::endl;
		}
	}

	return dags[bestDAG];
}

void Evaluation::evaluateWithRTDAandUpdate(const std::vector<DAG> &dags,
										   std::optional<std::reference_wrapper<DAG>> bestDag,
										   std::optional<std::reference_wrapper<float>> bestDagCost)
{
	std::vector<float> cost(dags.size(), 0.0);

	unsigned invalidDags = 0;

	for (unsigned k = 0; k < dags.size(); k++)
	{
		if (std::isnan(cost[k]))
			continue;

		SchedulingInfo info = getSchedulingInfo(dags[k], schedulingEval_.second);

		if (!schedulingEval_.second.isValid(info))
		{
			cost[k] = NAN;
			invalidDags++;
			continue;
		}

		cost[k] += schedulingEval_.first.getCost(info);
	}

	for (const auto &eval : latencyEval_)
	{
		std::vector<unsigned> chain = taskChainToNum(eval.first);

		for (unsigned k = 0; k < dags.size(); k++)
		{
			if (std::isnan(cost[k]))
				continue;

			auto info = getLatencyInfoRTDA(dags[k], chain);

			if (!eval.second.second.isValid(info))
			{
				cost[k] = NAN;
				invalidDags++;
				continue;
			}

			cost[k] += eval.second.first.getCost(info);
		}
	}
	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Num invalid dags: " << invalidDags << std::endl;
	}

	if (invalidDags == dags.size())
	{
		if (GlobalVariablesDAGOpt::debugMode)
		{
			std::cout << "No valid dag found. Constraints are too tight." << std::endl;
		}
		if (bestDag.has_value())
		{
			bestDag.value().get() = DAG{0};
		}
		if (bestDagCost.has_value())
		{
			bestDagCost.value().get() = std::numeric_limits<float>::max();
		}
		return;
	}

	unsigned bestDagId = 0;
	float minCost = std::numeric_limits<float>::max();
	for (unsigned k = 0; k < cost.size(); k++)
	{
		if (!std::isnan(cost[k]) && cost[k] < minCost)
		{
			bestDagId = k;
			minCost = cost[k];
		}
	}

	if (GlobalVariablesDAGOpt::debugMode)
	{
		std::cout << "Best DAG: " << bestDagId << ", with total cost: " << minCost << std::endl
				  << std::endl;
		for (const auto &eval : latencyEval_)
		{
			printChain(eval.first);
			auto info = getLatencyInfoRTDA(dags[bestDagId], taskChainToNum(eval.first));
			std::cout << info << std::endl;
		}
	}

	if (bestDag.has_value())
	{
		bestDag.value().get() = dags[bestDagId];
	}
	if (bestDagCost.has_value())
	{
		bestDagCost.value().get() = minCost;
	}
}

LatencyInfo Evaluation::getLatencyInfoRTDA(const DAG &dag, std::vector<unsigned> chain)
{
	LatencyInfo info;
	info.maxLatencyPair = std::make_pair((unsigned)0, (unsigned)0);
	info.minLatencyPair = std::make_pair((unsigned)0, (unsigned)0);
	info.reactionTimePair = std::make_pair((unsigned)0, (unsigned)0);

	RegularTaskSystem::TaskSet tasks = GetTaskSet(dag);
	RegularTaskSystem::TaskSetInfoDerived tasksInfo(tasks);
	VectorDynamic initialEstimate = GetInitialEstimate(dag, schedulingEval_.second.maxCores);
	// Values initialEstimateFG = OrderOptDAG_SPACE::GenerateInitialFG(initialEstimate, tasksInfo);
	std::vector<int> causeEffectChain(chain.begin(), chain.end());
	auto res = OrderOptDAG_SPACE::GetRTDAFromSingleJob(tasksInfo, causeEffectChain, initialEstimate);
	OrderOptDAG_SPACE::RTDA resM = OrderOptDAG_SPACE::GetMaxRTDA(res);
	info.maxLatency = resM.dataAge;
	info.reactionTime = resM.reactionTime;
	info.minLatency = 0.0f;
	return info;
}

void Evaluation::addScheduling(const SchedulingCost &cost, const SchedulingConstraint &constraint)
{
	schedulingEval_ = std::make_pair(cost, constraint);
}

std::vector<unsigned>
Evaluation::taskChainToNum(const Chain &chain)
{
	std::vector<unsigned> c;
	for (const auto &node : chain)
	{
		c.push_back(node->id - 1);
	}
	return c;
}

void Evaluation::printChain(const Chain &chain)
{
	std::cout << "Chain: ";

	for (const auto &n : chain)
	{
		std::cout << "->" << n->name;
	}
	std::cout << std::endl;
}

SchedulingInfo
Evaluation::getSchedulingInfo(const DAG &dag, const SchedulingConstraint &constraint)
{
	float u = dag.getOriginatingTaskset()->getUtilization();

	for (unsigned m = std::max(1.0f, std::ceil(u)); m <= constraint.maxCores; m++)
	{
		if (scheduling::scheduleDAG(dag, m))
			return SchedulingInfo(m);
	}

	return SchedulingInfo(constraint.maxCores + 1);
}
