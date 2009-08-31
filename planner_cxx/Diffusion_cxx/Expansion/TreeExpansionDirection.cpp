/*
 * TreeExpansion.cpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */

#include "TreeExpansionMethod.hpp"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"


shared_ptr<Configuration> TreeExpansionMethod::getExpansionDirection(Node* fromComp,
		Node* toComp,
		bool samplePassive,
		Node*& directionNode) {

	if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH) {

		shared_ptr<Configuration> q = mGraph->getRobot()->randDirShoot(samplePassive);

		p3d_addConfig(mGraph->getRobot()->getP3dRob(),
				q->getP3dConfigPt(),
				fromComp->getComp()->dist_nodes->N->q,
				q->getP3dConfigPt());

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
    q = shared_ptr<Configuration>(directionNode->getConf());
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

      p3d_shoot_inside_box(mGraph->getRobot()->getP3dRob(),
			   expandComp->getComp(),
			   q->getP3dConfigPt(),
			   expandComp->getComp()->box_env_small,
			   samplePassive);
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
