/*
 * DummyNodes.h
 *
 *  Created on: Apr 10, 2019
 *      Author: mirco
 */

#ifndef MULTIRATE_DUMMYNODES_H_
#define MULTIRATE_DUMMYNODES_H_
#include <memory>
#include <vector>

#include "sources/Baseline/Verucchi20/DAG/DAG.h"
#include "sources/Baseline/Verucchi20/DAG/Node.h"
#include "sources/Utils/Parameters.h"

struct DummyNodes
{

	void
	addToDAG(DAG &dag, unsigned hyperperiod);

	bool
	brokenDummyChain(const DAG &dag);

	std::vector<std::shared_ptr<Node>> dummyTasks;
	std::vector<std::shared_ptr<Node>> syncNodes;
	std::vector<Edge> dummyChain;
};

#endif /* MULTIRATE_DUMMYNODES_H_ */
