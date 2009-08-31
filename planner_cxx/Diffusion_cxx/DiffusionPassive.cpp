/*
 * PassiveDiffusion.cpp
 *
 *  Created on: Jun 30, 2009
 *      Author: jmainpri
 */
#include "RRT.hpp"

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "P3d-pkg.h"
#include "Bio-pkg.h"

#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <tr1/memory>
#include <algorithm>

#include "../../userappli/StatCostStructure.hpp"
#include "c_to_qt.h"

using namespace std;

int RRT::expandOneStepManhatan(Node* fromComp, Node* toComp){
	Node* directionNode(NULL);
	// get direction
	shared_ptr<Configuration> directionConfig;
	directionConfig = Expansion->getExpansionDirection(
			fromComp, toComp, false, directionNode);

	// get node for expansion toward direction
	Node* expansionNode(Expansion->getExpansionNode(fromComp,
			directionConfig,
			ACTIVE_CONFIG_DIST));

	// copy passive dofs
	expansionNode->getConf()->copyPassive(*directionConfig);
	// expand the active dofs
	int nbCreatedNodes = expandProcess(*expansionNode, directionConfig,
			directionNode, ENV.getExpansionMethod());
	// expand the passive dofs
	return (nbCreatedNodes + passiveExpandProcess(expansionNode,
			nbCreatedNodes, directionNode));
}

bool RRT::manhattanSamplePassive() {
	return(p3d_GetManhattanRatio() < p3d_random(0.,1.));
}

int RRT::passiveExpandProcess(Node* expansionNode,
		int NbActiveNodesCreated, Node* directionNode) {
	Node* lastCreatedNode;
	int nbPasExp(0);
	bool firstPass(true);
	bool reiterate(true);

	if((p3d_GetIsPasExtWhenAct()== FALSE)  && (NbActiveNodesCreated == 0)){
		/* The passive dof expansion only works if the
       active dofs have been expanded */
		return 0;
	}

	shared_ptr<Configuration> invalConf(new Configuration(mR));
	/*Warning: I don't anderstand the function of the Reiterate parameter */
	std::vector<p3d_jnt*> oldJoints;

	configPt invalC = invalConf->getP3dConfigPt();

	while(reiterate && p3d_ExpanBlockedByColl(mR->getP3dRob(), &invalC ))
	{
		reiterate = false;
		std::vector<p3d_jnt*> joints;
		if(getCollidingPassiveJntList(mR->getP3dRob(), invalConf->getP3dConfigPt(), joints))
		{
			std::vector<p3d_jnt*> newJoints;
			// select only the passive parameters that have not been expanded yet
			if(selectNewJntInList(mR->getP3dRob(), joints, oldJoints, newJoints))
			{
				if((NbActiveNodesCreated == 0)&& firstPass) {
					/* No node has been created during the active node expansion */
					lastCreatedNode = expansionNode;
				} else {
					lastCreatedNode = mG->getNode(mG->getGraph()->last_node->N);
				}
				firstPass = false;
				shared_ptr<Configuration> newRandConf(mG->lastNode()->getConf()->copy());
				bool expansionSucceeded(false);
				for(uint i(0); !expansionSucceeded; i++)
				{
					if(i >= p3d_GetMaxPassiveExpand())
						return(nbPasExp);

					shoot_jnt_list_and_copy_into_conf(mR->getP3dRob(),
							newRandConf->getP3dConfigPt(),
							newJoints);

					int nbCreatedNodes = expandProcess(*lastCreatedNode, newRandConf, directionNode, ENV.getExpansionMethod());
					if(nbCreatedNodes > 0)
					{
						reiterate = true;
						expansionSucceeded = true;
						nbPasExp += nbCreatedNodes;
						if(Manha_DEBUG)
							cout << "Expanded passive parameters at try " << i+1 << endl;
					}
				}
			}
		}
	}
	return(nbPasExp);
}
