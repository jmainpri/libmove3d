/*
 * Transition-RRT.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TRANSITIONRRT_HPP_
#define TRANSITIONRRT_HPP_

#include "../RRT.hpp"

class TransitionRRT : public RRT {

public:

	TransitionRRT(WorkSpace* WS);

	~TransitionRRT();

	int init();

	bool connectNodeToCompco(Node* N, Node* CompNode);

};

#endif /* TRANSITIONRRT_HPP_ */
