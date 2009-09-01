/*
 * TreeExpansion.cpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */

#include "TreeExpansionMethod.hpp"

using namespace std;
using namespace tr1;

shared_ptr<Configuration> TreeExpansionMethod::getExpansionDirection(Node* fromComp,
		Node* toComp,
		bool samplePassive,
		Node*& directionNode) {

	if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {

		shared_ptr<Configuration> q = mGraph->getRobot()->shootDir(samplePassive);

		p3d_addConfig(mGraph->getRobot()->getRobotStruct(),
				q->getConfigStruct(),
				fromComp->getCompcoStruct()->dist_nodes->N->q,
				q->getConfigStruct());

		return (q);

	} else {
		shared_ptr<Configuration> q = selectExpansionDirection(fromComp,
				toComp,
				samplePassive,
				directionNode);

		return (q);
	}
}

shared_ptr<Configuration> TreeExpansionMethod::selectExpansionDirection(Node* expandComp,
					Node* goalComp,
					bool samplePassive,
					Node*& directionNode) {

  shared_ptr<Configuration> q;
  int savedRlg;

  if(IsDirSampleWithRlg) {
    // Save the previous Rlg setting to shoot without Rlg
    savedRlg = p3d_get_RLG();
    p3d_set_RLG(false);
  }

  // Selection in the entire CSpace and
  // biased to the Comp of the goal configuration
  if(IsGoalBias &&
     p3d_random(0.,1.) <= GoalBias)
  {
    // select randomly a node in the goal component as direction of expansion
    directionNode = mGraph->randomNodeFromComp(goalComp);
    q = directionNode->getConfiguration();
  }
  else
  {
    switch(ExpansionDirectionMethod)
    {
    case SUBREGION_CS_EXP:
      // Selection in a subregion of the CSpace
      // (typically close to the current tree)
      // and  biased to the goal configuration
      q = shared_ptr<Configuration>(new Configuration(mGraph->getRobot()));

      p3d_shoot_inside_box(
    		  mGraph->getRobot()->getRobotStruct(),
    		  /*expandComp->getConfiguration()->getConfigStruct(),*/
    		  q->getConfigStruct(),
    		  expandComp->getCompcoStruct()->box_env_small,
    		  (int)samplePassive);
      break;

    case GLOBAL_CS_EXP:
    default:
      // Selection in the entire CSpace
      q = mGraph->getRobot()->shoot(samplePassive);
    }
  }
  if(!IsDirSampleWithRlg)
  {
    //Restore the previous Rlg setting
    p3d_set_RLG(savedRlg);
  }
  return(q);
}
