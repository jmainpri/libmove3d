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
    Cell* getNextCellOnPath(Vector3d pos);

    /**
      * Configuration from the next cell along the 3dPath
      */
    std::tr1::shared_ptr<Configuration> getConfigurationInNextCell(Node* node,bool foward);

 private:
    HRICS::Grid*            _3DGrid;
    std::vector<Vector3d>   _3DPath;

    Cell*                   _LastForward;
    Cell*                   _LastBackward;

};

#endif // HRICS_RRTEXPANSION_H
