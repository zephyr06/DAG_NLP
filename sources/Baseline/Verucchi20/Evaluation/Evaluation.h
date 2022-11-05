/*
 * Evaluation.h
 *
 *  Created on: May 3, 2019
 *      Author: mirco
 */

#ifndef EVALUATION_EVALUATION_H_
#define EVALUATION_EVALUATION_H_
#include <sources/Baseline/Verucchi20/DAG/DAG.h>
#include <sources/Baseline/Verucchi20/Evaluation/LatencyInfo.h>
#include <sources/Baseline/Verucchi20/Evaluation/SchedulingInfo.h>
#include <sources/Baseline/Verucchi20/MultiRate/MultiNode.h>

class Evaluation
{
public:
	using Chain = std::vector<std::shared_ptr<MultiNode>>;

	void
	addLatency(const Chain &chain, const LatencyCost &cost, const LatencyConstraint &constraint);

	void
	addScheduling(const SchedulingCost &cost, const SchedulingConstraint &constraint);

	const DAG &
	evaluate(const std::vector<DAG> &dags);

	const DAG &
	evaluateWithRTDA(const std::vector<DAG> &dags);

	void evaluateWithRTDAandUpdate(const std::vector<DAG> &dags,
						  std::optional<std::reference_wrapper<DAG>> bestDag  = std::nullopt,
						  std::optional<std::reference_wrapper<float>> minCost  = std::nullopt);

	LatencyInfo getLatencyInfoRTDA(const DAG &dag, std::vector<unsigned> chain);

	std::vector<unsigned>
	taskChainToNum(const Chain &chain);

	void
	printChain(const Chain &chain);

private:
	SchedulingInfo
	getSchedulingInfo(const DAG &dag, const SchedulingConstraint &constraint);

	std::vector<std::pair<Chain, std::pair<LatencyCost, LatencyConstraint>>> latencyEval_;

	std::pair<SchedulingCost, SchedulingConstraint> schedulingEval_;
};

#endif /* EVALUATION_EVALUATION_H_ */
