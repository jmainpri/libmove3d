/*
 * TreePlanner.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TREEPLANNER_HPP_
#define TREEPLANNER_HPP_

#include "../planner.hpp"

/**
  @ingroup NEW_CPP_MODULE
  @defgroup Diffusion
  @brief Tree planner module
  \image html RRT_graph2.png
  */

/**
  @ingroup Diffusion
  */
class TreePlanner : public Planner {

public:
	/**
	 * Constructor
	 */
        TreePlanner(Robot* R, Graph* G);

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
          * Main function to connect to the other Connected Component
          */
        bool connectionToTheOtherCompco(Node* toNode);

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
        unsigned int run();

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
        int mNbExpansion;

};

#endif /* TREEPLANNER_HPP_ */
