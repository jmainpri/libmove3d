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

int nb_connect_node_to_Comp = 0;

RRT::RRT(p3d_graph* G, int(*stop_func)(), void(*draw_func)()) :
	mConsecutiveFailures(0), _stop_func(stop_func), _draw_func(draw_func), mR(
			new Robot(G->rob)), mG(new Graph(G ? G : p3d_create_graph(), mR,
			new LocalpathFactory(), new SamplingAPI(mR))), mStart(NULL), mGoal(
			NULL) {

	shared_ptr<Configuration> startConf = mR->getRobotPos().copy();
	shared_ptr<Configuration> goalConf;

	// Check if goal is in collision
	if (ENV.getBool(Env::expandToGoal)) {
		goalConf = mR->getGoto().copy();
		if (mR->isInCollision(*goalConf)) {
			// TODO: exception
			cout
					<< "Diffusion process stopped: \
			Goal configuration in collision << "
					<< endl;
		}
	}

	// If startConf and/or goalConf are not in the graph, add them
	Node* nS(mG->searchConf(startConf));
	mStart = nS ? nS : mG->insertExtremalNode(startConf);

	cout << "---------- Start Configuration -----------" << endl;
	if (!nS) {
		cout << "Attention nStart was not in Graph" << endl;
	}
	mStart->print();

	if (ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal)) {
		Node* nG(mG->searchConf(goalConf));
		mGoal = nG ? nG : mG->insertExtremalNode(goalConf);
		cout << "----------- Goal Configuration -----------" << endl;
		if (!nG) {
			cout << "Attention nGoal was not in Graph" << endl;
		}
		mGoal->print();
	}

	Expansion = new TreeExpansionMethod(mG);
}

void RRT::reset() {
	delete (mG);
	delete (Expansion);
	//	RRT* ptrDiffision = new RRT(NULL,_stop_func,_draw_func);
	//	this = ptrDiffision;
}

RRT::~RRT() {
	delete (mG);
	delete (Expansion);
}

shared_ptr<Configuration> RRT::diffuseOneConf(
		shared_ptr<Configuration> qCurrent) {
	Localpath path(qCurrent, mR->shoot());
	return (path.confAtParam(std::min(path.length(), Expansion->step())));
}

void RRT::checkStopByWeight(Node* n) {
	double stopWeight;
	int signStopWeight;
	p3d_GetStopWeightAndSign(&stopWeight, &signStopWeight);
	if (signStopWeight * (n->mN->weight - stopWeight) > 0) {
		p3d_SetStopValue(true);
		p3d_SetDiffuStoppedByWeight(true);
	}
}

void RRT::connectNodeToComp(Node* node, Node* compNode) {

	if (p3d_GetIsCostFuncSpace()) {
		costConnectNodeToComp(node, compNode);
	} else {
		mG->connectNodeToComp(node, compNode);
	}

}

Node* RRT::connectNode(Node* currentNode, Localpath& path, double pathDelta,
		Node* directionNode, double currentCost, int& nbCreatedNodes) {
	if ((pathDelta == 1. && directionNode)) {
		p3d_MergeComp(mG->getGraph(), currentNode->mN, directionNode->mN,
				path.length());

		return (directionNode);
	} else {
		Node* newNode = insertNode(path.mEnd, currentNode, path.length(),
				currentCost);

		nbCreatedNodes++;
		return (newNode);
	}
}

Node* RRT::insertNode(shared_ptr<Configuration> q, Node* expansionNode,
		double expansionDist, double currentCost) {
	Node* node(mG->insertRrtLinkingNode(q, expansionNode, expansionDist));
	p3d_graph* g(mG->getGraph());

	// Cost updates
	if (p3d_GetIsCostFuncSpace()) {
		p3d_SetNodeCost(g, node->mN, currentCost);
		//for adaptive variant, new temp is refreshed except if it is going down.
		if (currentCost < expansionNode->mN->cost)
			node->mN->temp = expansionNode->mN->temp;
		else
			node->mN->temp = expansionNode->mN->temp / 2.;
	}

	//weight updates
	if (p3d_GetIsWeightedChoice())
		p3d_SetNodeWeight(g, node->mN);

	//check stop conditions
	if (p3d_GetIsWeightStopCondition())
		this->checkStopByWeight(node);

	// Graph updates for RANDOM_IN_SHELL method
	if (Expansion->getNodeMethod() == RANDOM_IN_SHELL_METH) {
		p3d_SetNGood(p3d_GetNGood() + 1);
		if (node->mN->weight > g->CurPbLevel) {
			g->CurPbLevel = node->mN->weight;
			g->n_consec_pb_level = 0;
			g->n_consec_fail_pb_level = 0;
			if (p3d_GetNGood() > 2)
				g->critic_cur_pb_level = g->CurPbLevel;
		} else {
			g->n_consec_pb_level++;
			g->n_consec_fail_pb_level = 0;
		}
	}

	//Additional cycles through edges addition if the flag is active
	if (ENV.getBool(Env::addCycles))
		addCycles(*node);
	return (node);
}

void RRT::addCycles(Node& newNode) {
	double longStep = 3. * Expansion->step();

	// List of closest Nodes
	p3d_list_node* listDistNodePt = p3d_listNodeInDist(mR->getP3dRob(),
			newNode.mN->comp, newNode.mN, longStep);

	p3d_list_node* savedListDistNodePt = listDistNodePt;

	// Loop on the list
	while (listDistNodePt) {

		// Keep only closest
		if (!p3d_IsSmallDistInGraph(mG->getGraph(), newNode.mN,
				listDistNodePt->N, 5, Expansion->step())) {

			Node& closeNode = *mG->getNode(listDistNodePt->N);
			std::tr1::shared_ptr<Configuration> closeConf = mG->getNode(
					listDistNodePt->N)->getConfSP();

			Localpath path = Localpath(newNode.getConfSP(), closeConf);

			if (path.valid() && costTestSucceeded(&newNode, *closeConf,
					closeConf->cost()) && costTestSucceeded(&closeNode,
					*(newNode.getConf()), closeConf->cost())) {
				// cout << "create a cycle edge" << endl;
				p3d_create_edges(mG->getGraph(), newNode.mN, listDistNodePt->N,
						path.length());
				newNode.mN->edges->E->for_cycle = true;
			}
		}
		listDistNodePt = listDistNodePt->next;
	}

	while (savedListDistNodePt) {
		p3d_list_node* destroyListNodePt = savedListDistNodePt;
		savedListDistNodePt = savedListDistNodePt->next;
		MY_FREE(destroyListNodePt, p3d_list_node, 1);
	}
}

bool RRT::checkStopConditions() {
	if (ENV.getBool(Env::expandToGoal) && mG->inSameComponent(mStart, mGoal)) {
		cout << "Success: the start and goal components are connected." << endl;
		return (true);
	}
	if (ENV.getBool(Env::ligandExitTrajectory)) {
		double d(mG->getRobot()->dist(*mStart->getConf(),
				*mG->lastNode()->getConf()));
		if (d > 12.0) {
			ENV.setBool(Env::expandToGoal, true);
			mGoal = mG->lastNode();
			mG->getGraph()->search_goal = mGoal->mN;
			mGoal->mN->rankFromRoot = 1;
			mGoal->mN->type = ISOLATED;
			/*p3d_copy_config_into(mG->getRobot()->getP3dRob(),
			 mGoal->getConf()->getP3dConfigPt(),
			 &mG->getRobot()->getGoto().getP3dConfigPt());*/
			cout << "BROKEN!!!!!!!" << endl;
			cout << "Success: distance from start is " << d << endl;
			return (true);
		}
	}
	if (mStart->getComp()->nnode >= p3d_get_COMP_NODES()) {
		cout
				<< "Failure: the maximum number of nodes in the start component is reached."
				<< endl;
		return (true);
	}
	if (ENV.getBool(Env::biDir))
		if (mGoal->getComp()->nnode >= p3d_get_COMP_NODES()) {
			cout
					<< "Failure: the maximum number of nodes in the goal component is reached."
					<< endl;
			return (true);
		}
	if (mConsecutiveFailures > p3d_get_NB_TRY()) {
		cout
				<< "Failure: the maximum number of consecutive failures to expand a component is reached."
				<< endl;
		p3d_SetStopValue(true);
		return (true);
	}
	if (!(*_stop_func)())
		p3d_SetStopValue(true);
	if (p3d_GetStopValue()) {
		cout << "RRT expansion cancelled." << endl;
		return (true);
	}
	return (false);
}

bool RRT::trajFound() {
	return (mGoal ? mG->inSameComponent(mStart, mGoal) : false);
}

void RRT::writeTrajectoryPdbs(std::string filename, double granularity,
		Dist* dist) {
	p3d_rob* robotPt(mG->getRobot()->getP3dRob());
	uint filenumber(0);
	int njnt(robotPt->njoints);

	double dmax(p3d_get_env_dmax());
	//  double dmax = p3d_get_env_graphic_dmax();

	if (robotPt->tcur == NULL) {
		std::cout << "RRT::writeTrajectoryPdbs: no current trajectory"
				<< std::endl;
		return;
	}

	shared_ptr<std::fstream>
			trajFile(open_file("traj.txt", std::ios_base::out));
	p3d_localpath* localpathPt(robotPt->tcur->courbePt);
	double* distances = MY_ALLOC(double, njnt+1);
	double currentDist(0);
	double nextDist(0);

	std::tr1::shared_ptr<Localpath> path;
	double param(0.);
	std::tr1::shared_ptr<Configuration> lastConfig;

	while (true) {
		bool changed(false);
		if (path.get() && (param > path->length() - EPS6
				&& localpathPt->next_lp)) {
			localpathPt = localpathPt->next_lp;
			param = 0;
			changed = true;
		}
		if (!path.get() || changed) {
			configPt
					q1(localpathPt->config_at_param(robotPt, localpathPt, 0.0));
			configPt q2(localpathPt->config_at_param(robotPt, localpathPt,
					localpathPt->range_param));
			path = std::tr1::shared_ptr<Localpath>(new Localpath(
					mG->getRobot()->copyConfig(q1), mG->getRobot()->copyConfig(
							q2)));
			free(q1);
			free(q2);
		}

		std::tr1::shared_ptr<Configuration> q(path->confAtParam(param));

		if (lastConfig.get())
			currentDist += dist->compute(*mG->getRobot(), *lastConfig, *q);

		// approximation : deplacement is always greater than wanted
		if (currentDist >= nextDist || (!localpathPt->next_lp && param
				>= path->length() - EPS6)) {
			std::stringstream ss;
			ss << filenumber;
			mG->getRobot()->setAndUpdate(*q);
			std::string wholename("./test/" + filename + ss.str() + ".pdb");
			move3d_to_pdb(robotPt, wholename.c_str(), filenumber,
					trajExpMinNONE);
#ifdef STOCHASTIC
			(*trajFile) << currentDist << " " << compute_energy() << " " << filenumber << std::endl;
#endif
			filenumber++;
			if (!localpathPt->next_lp && param >= path->length() - EPS6)
				break;
			nextDist += granularity;
		}

		for (int i(0); i <= njnt; i++) {
			distances[i] = dmax;
		}

		param += localpathPt->stay_within_dist(robotPt, localpathPt, param,
				FORWARD, distances);

		lastConfig = q;
	}

	MY_FREE(distances, double, njnt+1);
}

/**
 * ExpandProcess
 *  General function expanding a node toward a direction
 * of expansion. The mode of expansion depends of the
 * expansion choice selected.
 */
int RRT::expandProcess(Node& expansionNode,
		shared_ptr<Configuration> directionConfig, Node* directionNode,
		Env::expansionMethod method) {

	if (method == Env::costConnect) {
		return ExpandCostConnect(expansionNode, directionConfig, directionNode,
				method, false);
	}

	bool extensionSucceeded(false);
	bool failed(false);
	int nbCreatedNodes(0);
	Node* fromNode(&expansionNode);
	Node* extensionNode(NULL);
	shared_ptr<Localpath> directionLocalpath;
	double positionAlongDirection(0.);
	shared_ptr<Localpath> extensionLocalpath;
	double extensionCost(0.);
	bool firstIteration(true);

	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while (firstIteration || (method == Env::nExtend && !failed
			&& positionAlongDirection < 1.)) {

		directionLocalpath = shared_ptr<Localpath> (new Localpath(
				fromNode->getConfSP(), directionConfig));

		extensionSucceeded = Expansion->nextStep(*directionLocalpath,
				directionNode, positionAlongDirection, extensionLocalpath,
				method);

		failed |= !extensionSucceeded;

		if (failed) {
			int nbCollFail = ENV.getInt(Env::nbCollExpanFailed);
			nbCollFail++;
			ENV.setInt(Env::nbCollExpanFailed, nbCollFail);

			if (ENV.getBool(Env::printCollFail)) {
				cout << "nbCollFail = " << nbCollFail << endl;
			}
		}

		// Transition test for cost spaces, increase temperature in case of failure
		if (!failed && p3d_GetIsCostFuncSpace()) {
			extensionCost = extensionLocalpath->mEnd->cost();
			if (!costTestSucceeded(fromNode, *extensionLocalpath->mEnd,
					extensionCost)) {
				adjustTemperature(false, fromNode);
				failed = true;
				int nbCostFail = ENV.getInt(Env::nbCostTransFailed);
				nbCostFail++;
				ENV.setInt(Env::nbCostTransFailed, nbCostFail);
				if (ENV.getBool(Env::printCostFail))
					cout << "nbCostFail = " << nbCostFail << endl;
			}
		}

		if(method != Env::nExtend){
			// Expansion Control
			if (firstIteration && !failed) {
				if (ENV.getBool(Env::expandControl) && !Expansion->expandControl(
						*directionLocalpath, positionAlongDirection, expansionNode))
					failed = true;
			}
		}

		// Add node to graph if everything succeeded
		if (!failed) {
			extensionNode = connectNode(fromNode, *extensionLocalpath,
					positionAlongDirection, directionNode, extensionCost,
					nbCreatedNodes);
			// In cost space, decrease temperature if the accepted cost is higher
			if (p3d_GetIsCostFuncSpace() && extensionCost > fromNode->getCost()) {
				adjustTemperature(true, extensionNode);
			}

			// costTemp
			//fprintf(traj_file, "%d, %5.6f, %5.20f\n", extensionNode->mN->num, extensionCost , extensionNode->getComp()->temperature);
		}

		if (firstIteration && failed) {
			Expansion->expansionFailed(expansionNode);
		}

		fromNode = extensionNode;
		firstIteration = false;
	}

	if (p3d_GetIsCostFuncSpace() && (p3d_GetCostMethodChoice()
			== MAXIMAL_THRESHOLD)) {
		p3d_updateCostThreshold();
	}

	return nbCreatedNodes;
}

/**
 * expand One Step
 *  General function expanding a node toward a direction
 * of expansion. The mode of expansion depends of the
 * expansion choice selected.
 */
int RRT::expandOneStep(Node* fromComp, Node* toComp) {

	if (ENV.getBool(Env::enableHri)) {
		hri_zones.setNodes(fromComp, toComp);
	}
	// ML-RRT expansion case
	if (ENV.getBool(Env::isManhattan) && !manhattanSamplePassive()) {
		return expandOneStepManhatan(fromComp, toComp);
	}
	// Standard expansion case
	else {
		// get node for expansion toward direction
		Node* expansionNode(NULL);
		Node* directionNode(NULL);
		shared_ptr<Configuration> directionConfig;

		directionConfig = Expansion->getExpansionDirection(fromComp, toComp,
				true, directionNode);

		//		cout << "----------------------------------" << endl;
		//		cout << "directionConfig =" << endl;
		//		directionConfig->print();

		int nbQRand = ENV.getInt(Env::nbQRand);
		nbQRand++;
		if (ENV.getBool(Env::printNbQRand)) {
			cout << "nbQRand = " << nbQRand << endl;
		}
		ENV.setInt(Env::nbQRand, nbQRand);

		// get node for expansion toward direction
		expansionNode = Expansion->getExpansionNode(fromComp, directionConfig,
				p3d_GetDistConfigChoice());

		// expansion
		return expandProcess(*expansionNode, directionConfig, directionNode,
				ENV.getExpansionMethod());
	}
}

uint RRT::run() {

	p3d_InitRun(mG->getGraph(), mStart->mN, mGoal ? mGoal->mN : NULL);

	int costEnv = p3d_GetIsCostFuncSpace();

	if (costEnv && (ENV.getExpansionMethod() == Env::connect)) {
		cout
				<< "Warning: Connect expansion strategy \
		is usually unsuited for cost spaces\n"
				<< endl;
		p3d_SetIsCostFuncSpace(FALSE);
	}

	if ((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal))
			&& mStart->getConf()->equal(*mGoal->getConf())) {
		cout << "Tree Expansion failed: root nodes are the same" << endl;
		return (0);
	}

	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;
	nb_connect_node_to_Comp = 0;
	Node* fromNode = mStart;
	Node* toNode = mGoal;

	if (ENV.getBool(Env::expandToGoal))
		connectNodeToComp(mGoal, mStart);

	while (!checkStopConditions()) {
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
				&& (fromNode->getComp()->nnode > toNode->getComp()->nnode + 2))) {

			//			cout << "----------------------------------" << endl;
			//			toNode->print();

			NbCurCreatedNodes = ENV.getBool(Env::biDir) ? expandOneStep(
					fromNode, toNode) : expandOneStep(mStart, mGoal);

			//					cout << "NbCurCreatedNodes = " << NbCurCreatedNodes << endl;

			if (NbCurCreatedNodes > 0) {
				(*_draw_func)();

				NbTotCreatedNodes += NbCurCreatedNodes;
				mConsecutiveFailures = 0;

				if (ENV.getBool(Env::expandToGoal)) {
					// TODO : check the (non?)-validity of the following 2 lines
					// this->connectNodeToComp(mGoal, mStart);

					//					cout << "Last Node: " << endl;
					//					mG->lastNode()->print();
					//
					//					cout << "----------------------------------" << endl;
					//					cout << "toNode: " << endl;
					//					toNode->print();

					connectNodeToComp(mG->lastNode(),
							ENV.getBool(Env::biDir) ? toNode : mGoal);

					//					mG->print();
				}

				//				nb_connect_node_to_Comp++;
				//				cout << "Connect_Node = " << nb_connect_node_to_Comp << endl;
			} else {
				mConsecutiveFailures++;
			}
		}
		if (ENV.getBool(Env::biDir)) {
			std::swap(fromNode, toNode);
		}
	}

	if (costEnv) {
		p3d_SetIsCostFuncSpace(TRUE);
	}

	(*_draw_func)();
	return (NbTotCreatedNodes);
}

RRT* rrt;
FILE* traj_file;

bool p3d_RunDiffusion(p3d_graph* GraphPt, int(*fct_stop)(void),
		void(*fct_draw)(void)) {
	cout << endl << "**************************************" << endl
			<< " Beginning of Diffusion search process" << endl << endl;

	MY_ALLOC_INFO("Before the graph creation");
	double tu, ts;
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	p3d_rob* RobotPt = GraphPt->rob;

	// CostTemp File & Stat File
	traj_file = fopen("costTemp.csv", "w");
	//	StatCost Stats(GraphPt,RobotPt);

	int nbAddedNodes = 0;
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 3, false);
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 4, false);
	p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 5, false);

	ChronoOn();

	rrt = new RRT(GraphPt, fct_stop, fct_draw);

	nbAddedNodes = rrt->run();

	ChronoPrint("");
	ChronoTimes(&tu, &ts);
	GraphPt->time = GraphPt->time + tu;

	ChronoOff();

	cout << endl << " End of Diffusion search process" << endl
			<< "**************************************" << endl << endl;

	if (!ENV.getBool(Env::expandToGoal))
		return false;

	bool res = rrt->trajFound();

	if (!res) {
		cout
				<< "No solution path: the exploration didn't \
		link a start and a goal configuration."
				<< endl;
	} else {
		// on construit la trajectoire entre les points etapes
		if (p3d_graph_to_traj(RobotPt)) {
			g3d_add_traj("Globalsearch", p3d_get_desc_number(P3D_TRAJ));

			//if (p3d_GetIsCostFuncSpace()) {
				Trajectory newTraj(new Robot(RobotPt),p3d_ExtractBestTraj(GraphPt));
				trajToDraw.resize(1);
				trajToDraw.at(0) = newTraj;
			//}

		} else {
			// printf("Problem during trajectory extraction\n");

		}
		g3d_draw_allwin_active();
	}
	cout << endl;

	// costTemp
	fclose(traj_file);
	//	Stats.setValues();
	//	Stats.print();

	PrintInfo(("'costTemChro.csv' creation\n"));

	return (res);
}
