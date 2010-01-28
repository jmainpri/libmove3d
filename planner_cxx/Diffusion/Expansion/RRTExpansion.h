#ifndef RRTEXPANSION_HPP
#define RRTEXPANSION_HPP

#include "BaseExpansion.h"

/**
  @ingroup Diffusion
  * RRT Expansion Methods
  *
  * Functions to expand a tree in the CSpace
  * Collision Checking, Random sampling and Biasing
  */
class RRTExpansion: public BaseExpansion
{

public:

    RRTExpansion();
    RRTExpansion(Graph* prtGraph);

    ~RRTExpansion();


    /**
     * Shoots a direction (includes the biasing)
     *
     * @param Expanding component
     * @param Goal Component
     * @param Sampling passive mode
     * @param Direction node
     */
    virtual std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive,
            Node*& directionNode);

    /**
     * Gets the nearest node in the graph
     *
     * @param compNode Expanding component
     * @param direction Direction of expansion
     * @param Sampling passive mode
     */
    virtual Node* getExpansionNode(
            Node* compNode, std::tr1::shared_ptr<Configuration> direction, int distance);

    /**
      * Basic Expansion Method
      *
      * @param expansionNode
      * @param directionConfig
      * @param directionNode
      * @param method
      */
    int expandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig,
                       Node* directionNode,
                       Env::expansionMethod method);

    /**
     * Expands towards the goal
     *
     * @param expansionNode     Node expanded
     * @param directionConfig   Direction
     */
    virtual bool expandToGoal(Node* expansionNode,
                      std::tr1::shared_ptr<Configuration> directionConfig);

};

#endif // RRTEXPANSION_HPP
