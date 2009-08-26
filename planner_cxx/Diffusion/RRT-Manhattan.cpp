/*
 * Manhattan.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "RRT.hpp"

using namespace std;
using namespace tr1;

#define ML_DEBUG 0

int RRT::selectNewJntInList(p3d_rob *robotPt, vector<p3d_jnt*>& joints,
		vector<p3d_jnt*>& oldJoints, vector<p3d_jnt*>& newJoints)
{
	for(uint i(0); i < joints.size(); i++)
	{
		bool found(false);
		for(uint j(0); j < oldJoints.size(); j++)
		{
			if(oldJoints[j] == joints[i])
			{
				found = true;
				break;
			}
		}
		if(!found)
		{
			newJoints.push_back(joints[i]);
			oldJoints.push_back(joints[i]);
		}
	}
	return(newJoints.size() > 0);
}

int RRT::getCollidingPassiveJntList(p3d_rob *robotPt, configPt qinv,
		vector<p3d_jnt*>& joints)
{
	p3d_poly* polys[2];

	// with BIO module
	if(p3d_col_get_mode() == p3d_col_mode_bio) {
		//    p3d_jnt** passiveJoints;
		//    int nJoints;
		//    bio_get_list_of_passive_joints_involved_in_collision(
		//      robotPt, qinv, &nJoints, &passiveJoints);
		//    for(int i(0); i < nJoints; i++)
		//      joints.push_back(passiveJoints[i]);
		//    MY_FREE(passiveJoints, p3d_jnt*, nJoints);
	}
	// without BIO module
	else
	{
		p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qinv);
		if(p3d_col_test() <= 0)
		{
			cout << "No collision detected" << endl;
			return(false);
		}

		// NOTE: KCD only retuns the first collision pair !!!
		// NOTE: ONLY THE PASSIVE JOINT INVOLVED IN THE COLLISION IS RETURNED
		//       BUT ALL THE PARENT JOINTS SHOULD BE ALSO CONSIDERED ???
		p3d_col_get_report(0,&polys[0],&polys[1]);
		for(uint i(0); i < 2; i++)
			if(polys[i]->p3d_objPt->jnt != NULL)
				if(!p3d_jnt_get_is_active_for_planner(polys[i]->p3d_objPt->jnt))
					joints.push_back(polys[i]->p3d_objPt->jnt);
	}
	return(joints.size() > 0);
}

void RRT::shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qrand,
		vector<p3d_jnt*>& joints)
{
	double perturb = 0.1; // NOTE: THIS SHOULD BE A PARAMETER

	// NOTE : the random shoot should be centered at q_inv !!!
	//        (doesn't matter in the case of "circular" joints)

	for(uint i(0); i < joints.size(); i++)
	{
		p3d_jnt* joint(joints[i]);
		for(int j(0); j < joint->dof_equiv_nbr; j++)
		{
			double vmin, vmax;
			double val, rval;
			p3d_jnt_get_dof_rand_bounds(joint, j, &vmin, &vmax);
			int k(joint->index_dof + j);
			if(!p3d_jnt_is_dof_circular(joint, j))
				val = p3d_random(vmin,vmax);
			else
			{
				double midrange = (vmax-vmin) / 2.0;
				// perturbation factor
				midrange *= perturb;
				rval = p3d_random(-midrange,midrange);
				val = qrand[k] + rval;
				val = MAX(MIN(val, vmax), vmin);
			}
			qrand[k] = val;
		}
	}
}

int RRT::passiveExpandProcess(Node* expansionNode, int NbActiveNodesCreated,
		Node* directionNode)
{
	Node* lastCreatedNode;
	int nbPasExp(0);
	bool firstPass(true);
	bool reiterate(true);

	if ((ENV.getBool(Env::isPasExtWhenAct) == FALSE) && (NbActiveNodesCreated
			== 0))
	{
		/* The passive dof expansion only works if the
		 active dofs have been expanded */
		return 0;
	}

	shared_ptr<Configuration> invalConf = shared_ptr<Configuration> (
			new Configuration(_Robot));
	/*Warning: I don't anderstand the function of the Reiterate parameter */
	vector<p3d_jnt*> oldJoints;
	//   while(reiterate && p3d_ExpanBlockedByColl(_Robot->get_robot(), invalConf->get_configPt()))
	//   {
	while (reiterate)
	{

		configPt q = invalConf->getConfigStruct();

		p3d_copy_config_into(_Robot->getRobotStruct(),
				_Robot->getRobotStruct()->currect_q_inv, &q);

		reiterate = false;
		vector<p3d_jnt*> joints;
		if (getCollidingPassiveJntList(_Robot->getRobotStruct(),
				invalConf->getConfigStruct(), joints))
		{
			vector<p3d_jnt*> newJoints;
			// select only the passive parameters that have not been expanded yet
			if (selectNewJntInList(_Robot->getRobotStruct(), joints, oldJoints,
					newJoints))
			{
				if ((NbActiveNodesCreated == 0) && firstPass)
				{
					/* No node has been created during the active node expansion */
					lastCreatedNode = expansionNode;
				}
				else
				{
					lastCreatedNode = _Graph->getNode(
							_Graph->getGraphStruct()->last_node->N);
				}
				firstPass = false;
				shared_ptr<Configuration> newRandConf =
						(_Graph->getLastnode()->getConfiguration()->copy());
				bool expansionSucceeded(false);
				for (int i(0); !expansionSucceeded; i++)
				{
					if (i >= ENV.getInt(Env::MaxPassiveExpand))
						return (nbPasExp);
					shoot_jnt_list_and_copy_into_conf(_Robot->getRobotStruct(),
							newRandConf->getConfigStruct(), newJoints);
					int nbCreatedNodes = expandProcess(lastCreatedNode,
							newRandConf, directionNode,
							ENV.getExpansionMethod());
					if (nbCreatedNodes > 0)
					{
						reiterate = true;
						expansionSucceeded = true;
						nbPasExp += nbCreatedNodes;
						if (ML_DEBUG)
							cout << "Expanded passive parameters at try " << i
									+ 1 << endl;
					}
				}
			}
		}
	}
	return (nbPasExp);
}

bool RRT::manhattanSamplePassive()
{
	return (ENV.getDouble(Env::manhatRatio) < p3d_random(0., 1.));
}

int RRT::passiveExpandOneStep(Node* fromComp,Node* toComp)
{
    Node* directionNode(NULL);
    Node* expansionNode(NULL);
    shared_ptr<Configuration> directionConfig;

	// get direction
	directionConfig = _expan->getExpansionDirection(fromComp,toComp,
			false,directionNode);
	// get node for expansion toward direction
	expansionNode = _expan->getExpansionNode(fromComp,directionConfig,
			ACTIVE_CONFIG_DIST);
	// copy passive dofs
	expansionNode->getConfiguration()->copyPassive(*directionConfig);
	// expand the active dofs
	int nbCreatedNodes = expandProcess(expansionNode,
			directionConfig, directionNode, ENV.getExpansionMethod());
	// expand the passive dofs
	return (nbCreatedNodes + this->passiveExpandProcess(expansionNode,
			nbCreatedNodes, directionNode));
}
