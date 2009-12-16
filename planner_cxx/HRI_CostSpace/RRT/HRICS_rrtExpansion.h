#ifndef HRICS_RRTEXPANSION_H
#define HRICS_RRTEXPANSION_H

#include "../../API/planningAPI.hpp"
#include "../../Diffusion/Expansion/TransitionExpansion.h"
#include "../HRICS_Grid.h"


class HRICS_rrtExpansion : public TransitionExpansion
{
public:
    HRICS_rrtExpansion();
    HRICS_rrtExpansion(Graph* G);

    /**
      * Direction used in RRT one step
      */
    std::tr1::shared_ptr<Configuration> getExpansionDirection(
        Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);

    /**
      * Next cell on 3d Path
      */
    API::Cell* getLastCellOnPath(std::vector<Vector3d> nodes);

    /**
      * Configuration from the next cell along the 3dPath
      */
    std::tr1::shared_ptr<Configuration> getConfigurationInNextCell(Node* node,bool foward);

    /**
      * Adds a node to a conected component
      */
    Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
                  Node* directionNode, int& nbCreatedNodes);

    /**
      * Checks it the cell is after the given cell on the
      * 3D path
      */
    bool afterAndOnPath(API::Cell* cell);

 private:
    HRICS::Grid*            _3DGrid;
    std::vector<Vector3d>   _3DPath;

    API::Cell*               _LastForward;
    API::Cell*               _LastBackward;

    bool                    _foward;

};

#endif // HRICS_RRTEXPANSION_H
