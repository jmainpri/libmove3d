/*
 * TreeExpansionNode.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */

#include "TreeExpansionMethod.hpp"

using namespace std;
using namespace tr1;

Node* TreeExpansionMethod::getExpansionNode(Node* compNode,
		shared_ptr<Configuration> direction,int distance) {

	if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
	{
			return mGraph->getNode(compNode->getCompcoStruct()->dist_nodes->N);
	}
	else
	{
			return selectExpansionNode(compNode, direction,distance);
	}
}

Node* TreeExpansionMethod::selectExpansionNode(Node* compNode,
		shared_ptr<Configuration> direction, int distance)
{
  int KNearest = -1;
  int NearestPercent;

 switch(distance) {

 case NEAREST_EXP_NODE_METH:
   /* Choose the nearest node of the componant*/
   return(mGraph->nearestWeightNeighbour(compNode,
		   direction,
		   p3d_GetIsWeightedChoice(),
		   distance));

 case K_NEAREST_EXP_NODE_METH:
   /* Select randomly among the K nearest nodes of a componant */
   NearestPercent = kNearestPercent;
   KNearest = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
   // TODO : fix
   //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
   // KNearest);

   return(mGraph->nearestWeightNeighbour(compNode,
		   direction,
		   p3d_GetIsWeightedChoice(),
		   distance));

 case BEST_SCORE_EXP_METH:
   /* Select the node which has the best score: weight*dist */
   return(mGraph->nearestWeightNeighbour(compNode,
		   direction,
		   p3d_GetIsWeightedChoice(),
		   distance));

 case K_BEST_SCORE_EXP_METH:
   NearestPercent = kNearestPercent;
   KNearest = MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
   // TODO : fix
   // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
   return(mGraph->nearestWeightNeighbour(compNode,
		   direction,
		   p3d_GetIsWeightedChoice(),
		   distance));

 case RANDOM_IN_SHELL_METH:
   /* Select randomly among all the nodes inside a given portion of shell */
   return(mGraph->getNode(hrm_selected_pb_node(mGraph->getGraphStruct(),
					   direction->getConfigStruct(),
					   compNode->getNodeStruct()->comp)));

 case RANDOM_NODE_METH:
   return(mGraph->getNode(p3d_RandomNodeFromComp(compNode->getCompcoStruct())));

 default:
   /* By default return the nearest node of the componant */
   return(mGraph->nearestWeightNeighbour(compNode,
		   direction,
		   p3d_GetIsWeightedChoice(),
		   distance));
  }
}
