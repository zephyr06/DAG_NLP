/*
 * VariableMultiEdge.h
 *
 *  Created on: May 2, 2019
 *      Author: mirco
 */

#ifndef VARIABLETASKSET_VARIABLEMULTIEDGE_H_
#define VARIABLETASKSET_VARIABLEMULTIEDGE_H_
#include <sources/Baseline/Verucchi20/MultiRate/MultiEdge.h>
#include <sources/Baseline/Verucchi20/MultiRate/MultiNode.h>

struct VariableMultiEdge
{

	std::shared_ptr<MultiNode> from;
	std::shared_ptr<MultiNode> to;
	std::vector<unsigned> jitter;

	std::vector<MultiEdge>
	translateToMultiEdges();
};

#endif /* VARIABLETASKSET_VARIABLEMULTIEDGE_H_ */
