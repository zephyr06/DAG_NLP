/*
 * Edge.cpp
 *
 *  Created on: Apr 9, 2019
 *      Author: mirco
 */
#include "sources/Baseline/Verucchi20/DAG/Edge.h"
#include "sources/Baseline/Verucchi20/DAG/Node.h"
#include "string"

Edge::Edge()
{
}

Edge::Edge(std::shared_ptr<Node> f, std::shared_ptr<Node> t) : from(f), to(t)
{
}

void Edge::flipEdge()
{
	auto temp = from;
	from = to;
	to = temp;
}
