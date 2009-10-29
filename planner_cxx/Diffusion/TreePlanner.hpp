/*
 * TreePlanner.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TREEPLANNER_HPP_
#define TREEPLANNER_HPP_

#include "../planner.hpp"
#include "Expansion/BaseExpansion.hpp"

class TreePlanner : public Planner {

public:
	/**
	 * Constructor
	 */
	TreePlanner(WorkSpace* WS);

	/**
	 * Destructor
	 */
	~TreePlanner();

	/**
	 * Initializes Planner
	 */
	virtual int init();

	/**
	 * Checks out the Stop condition
	 */
	virtual bool checkStopConditions();

	/**
	 * Checks out the preconditions
	 */
	virtual bool preConditions();

	/**
	 * Tries to connect a node to the other
	 * connected component of the graph
	 *
	 * @param currentNode The node that will be connected
	 * @param ComNode The Connected Component
	 */
	virtual bool connectNodeToCompco(Node* N, Node* CompNode);

	/**
	 * Expands tree from component fromComp,
	 * to component toComp
	 * @param fromComp the starting connex component
	 * @param toComp the arriving connex component
	 * @return the number of node created
	 */
	virtual int expandOneStep(Node* fromComp, Node* toComp) = 0 ;

	/**
	 * Main function of the Tree process
	 * @return the number of Nodes added to the Graph
	 */
	uint run();

	/**
	 * Returns number of consecutive failure
	 * during plannification
	 */
	int getNumConsecutiveFail()
	{
		return _nbConscutiveFailures;
	};


protected:

	int _nbConscutiveFailures;

};

#endif /* TREEPLANNER_HPP_ */
