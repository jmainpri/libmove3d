//
// C++ Implementation: vis_prm
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "Visibility.hpp"

using namespace std;
using namespace tr1;

Vis_PRM::Vis_PRM(Robot* R, Graph* G)
        : PRM(R,G)
{
}

Vis_PRM::~Vis_PRM()
{
}

uint Vis_PRM::expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void))
{
    if(ENV.getBool(Env::expandToGoal) &&
       _Start->getConfiguration()->equal(*_Goal->getConfiguration()))
    {
        cout << "graph creation failed: start and goal are the same" << endl;
        return(0);
    }

    int nbAddedNode = 0;

    while(!this->checkStopConditions())
    {
        _Graph->createOneOrphanLinking(Graph_Pt, fct_draw, 2, &nbAddedNode, &_nbConscutiveFailures);
    }

    return nbAddedNode;
}
