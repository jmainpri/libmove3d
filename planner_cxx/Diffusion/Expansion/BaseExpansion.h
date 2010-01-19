/*
 * BaseExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_BASE_EXPANSION_HPP
#define P3D_BASE_EXPANSION_HPP

#include "../../../planner_cxx/API/planningAPI.hpp"

/**
  @ingroup Diffusion
  */
class BaseExpansion {

public:

    /**
      * Constructor
      */
    BaseExpansion();

    BaseExpansion(Graph* prtGraph);

    /**
      * Destructor
      */
    ~BaseExpansion();

    int getNodeMethod() { return ExpansionNodeMethod; }

    /**
      * Expansion Step (Delta)
      */
    double step();

    /**
      * Function called when a node can not be connected
      * @param the node which has not been connected
      */
    void expansionFailed(Node& node);

    /**
      * Adds a node to a connected component
      */
    virtual Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
                  Node* directionNode, int& nbCreatedNodes);

    /**
      * Function that balances the ratio of
      * Exploration towards refinement
      */
    bool expandControl(LocalPath& path,
                       double positionAlongDirection,
                       Node& compNode);

    /**
      * Returns a configuration on the local path
      */
    bool nextStep(LocalPath& path,
                  std::tr1::shared_ptr<Configuration>& directionConfig,
                  double& pathDelta,
                  std::tr1::shared_ptr<LocalPath>& newPath,
                  Env::expansionMethod method);

    /**
      * Returns a configuration on the local path
      */
    bool nextStep(LocalPath& path,
                  Node* directionNode,
                  double& pathDelta,
                  std::tr1::shared_ptr<LocalPath>& newPath,
                  Env::expansionMethod method);

    /**
     * expandProcess
     * @param expansionNode
     * @param directionConfig
     * @param directionNode
     * @param method
     * @return
     */
    virtual int expandProcess(Node* expansionNode,
                              std::tr1::shared_ptr<Configuration> directionConfig,
                              Node* directionNode, Env::expansionMethod method) = 0;

protected:

    int ExpansionNodeMethod;
    int MaxExpandNodeFailure;
    int kNearestPercent;

    int ExpansionDirectionMethod; // = GLOBAL_CS_EXP;
    //	double GoalBias; //= 0.1;
    //	bool IsGoalBias; //= FALSE;
    bool IsDirSampleWithRlg; //= FALSE;

    Graph* mGraph;
};

#endif
