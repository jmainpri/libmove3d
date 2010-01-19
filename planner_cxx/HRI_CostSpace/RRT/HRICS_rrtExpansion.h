#ifndef HRICS_RRTEXPANSION_H
#define HRICS_RRTEXPANSION_H

#include "../../API/planningAPI.hpp"
#include "../../Diffusion/Expansion/TransitionExpansion.h"
#include "../Grid/HRICS_Grid.h"

/**
  @ingroup HRICS
  @brief Special RRT Expansion method for the HRICS
  */
class HRICS_rrtExpansion : public TransitionExpansion
{
public:
    HRICS_rrtExpansion();
    HRICS_rrtExpansion(Graph* G);

    /**
      * Initializes some variables for the expansion
      * method
      */
    void init();

     /**
      * Sets the grid
      */
    void setGrid(API::Grid* grid) { _3DGrid = dynamic_cast<HRICS::Grid*>(grid); }

     /**
      * Sets the cell path
      */
    void setCellPath(std::vector<API::Cell*> cellPath);

    /**
      * Direction used in RRT one step
      */
    std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);

    /**
      * Configuration from the next cell along the 3dPath
      */
    std::tr1::shared_ptr<Configuration> getConfigurationInNextCell(Node* node);

    /**
      * Adds a node to a conected component
      */
    Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
                  Node* directionNode, int& nbCreatedNodes);

    /**
      * Checks it the cell is after the given cell on the
      * 3D path
      */
    bool on3DPathAndAfter(API::Cell* cell);

private:
    HRICS::Grid*             _3DGrid;
    std::vector<API::Cell*>  _3DCellPath;

    API::Cell*               _LastForward;
    API::Cell*               _LastBackward;

    bool                     _forward;
    bool                     _biasing;

    double*                  _Box;

};

extern API::Cell* BiasedCell;

#endif // HRICS_RRTEXPANSION_H
