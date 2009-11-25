//
// C++ Implementation: acr
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ACR.hpp"

using namespace std;
using namespace tr1;

ACR::ACR(Robot* R, Graph* G)
 : PRM(R,G)
{
}

ACR::~ACR()
{
}

uint ACR::expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void))
{
  if(ENV.getBool(Env::expandToGoal) &&
     _Start->getConfiguration()->equal(*_Goal->getConfiguration()))
  {
    cout << "graph creation failed: start and goal are the same" << endl;
    return(0);
  }

  int nbAddedNode = 0;

  while(!this->checkStopConditions(*fct_stop))
  {
    shared_ptr<Configuration> C = _Robot->shoot();
    _nbConscutiveFailures ++;
    if (!C->IsInCollision())
    {
      _nbConscutiveFailures = 0;
      Node* N = new Node(_Graph,C);
      _Graph->insertNode(N);
      _Graph->linkToAllNodes(N);

      nbAddedNode ++;
      if (ENV.getBool(Env::drawGraph))
      {
        *Graph_Pt = *(_Graph->getGraphStruct());
	(*fct_draw)();
      }
    }
//     else
//     {
//       C->Clear();
//       C->~Configuration();
//     }
  }
  *Graph_Pt = *(_Graph->getGraphStruct());
  return nbAddedNode;
}
