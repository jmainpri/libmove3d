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

    TransitionRRT(Robot* R, Graph* G);

    ~TransitionRRT();

    int init();

    /**
    * costConnectNodeToComp
    * Try to connect a node to a given component
    * taking into account the fact that the space
    * is a cost space
    * @return: TRUE if the node and the componant have
    * been connected.
    */
    virtual bool connectNodeToCompco(Node* N, Node* CompNode);

};

#endif /* TRANSITIONRRT_HPP_ */
