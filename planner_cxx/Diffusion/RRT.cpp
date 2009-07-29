//
// C++ Implementation: rrt
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "RRT.hpp"

using namespace std;
using namespace tr1;

#define ML_DEBUG 0

RRT::RRT(WorkSpace* WS)
: Planner(WS)
{
	_nbConscutiveFailures = 0;
}

RRT::~RRT()
{
}

int RRT::init()
{
	int ADDED = 0;
	Planner::init();
	_nbConscutiveFailures = 0;
	ADDED += Planner::setStart(_Robot->getInitialPosition());
	ADDED += Planner::setGoal(_Robot->getGoTo());
	_Init = true;

	return ADDED;
}

int RRT::getNbFailures()
{
	return _nbConscutiveFailures;
}

bool RRT::checkStopConditions(int (*fct_stop)(void))
{
	if(ENV.getBool(Env::expandToGoal) &&
			trajFound())
	{
		cout << "Success: the start and goal components are connected." << endl;
		return(true);
	}
	if(/*ENV.getBool(Env::ligandExitTrajectory)*/0)
	{
		double d(_Start->getConfiguration()->dist(*_Graph->getLastnode()->getConfiguration()));
		if(d > 12.0)
		{
			ENV.setBool(Env::expandToGoal, true);
			_Goal = _Graph->getLastnode();
			_Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
			_Goal->getNodeStruct()->rankFromRoot = 1;
			_Goal->getNodeStruct()->type = ISOLATED;
			_Robot->getGoTo() = _Goal->getConfiguration()->copy();
			cout << "Success: distance from start is " << d << endl;
			return(true);
		}
	}

	if(_Start->maximumNumberNodes())
	{
		cout << "Failure: the maximum number of nodes in the start component is reached." << endl;
		return(true);
	}

	if(ENV.getBool(Env::biDir))
	{
		if(_Goal->maximumNumberNodes())
		{
			cout << "Failure: the maximum number of nodes in the goal component is reached." << endl;
			return(true);
		}
	}

	if(_Graph->getNbNode() >= ENV.getInt(Env::maxNodeCompco)  )
	{
		cout << "Failure: the maximum number of nodes is reached." << endl;
		return(true);
	}

	if(_nbConscutiveFailures > ENV.getInt(Env::NbTry))
	{
		cout << "Failure: the maximum number of consecutive failures to expand a component is reached." << endl;
		return(true);
	}

	if (!(*fct_stop)())
		p3d_SetStopValue(true);
	if(p3d_GetStopValue())
	{
		cout << "RRT expansion cancelled." << endl;
		return(true);
	}
	return(false);
}

shared_ptr<Configuration> RRT::diffuseOneConf(shared_ptr<Configuration> qCurrent) {
	shared_ptr<LocalPath> path = shared_ptr<LocalPath>(new LocalPath(qCurrent, _Robot->shoot()));
	return(path->configAtParam(_Robot, std::min(path->Length(), this->step())));
}

int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
	Node* directionNode(NULL);
	Node* expansionNode(NULL);
	shared_ptr<Configuration> directionConfig;

	// ML-RRT expansion case
	if(ENV.getBool(Env::isManhattan) && !(this->manhattanSamplePassive()))
	{
		// get direction
		directionConfig = fromComp->getExpansionDirection(toComp, false, directionNode);
		// get node for expansion toward direction
		expansionNode = fromComp->getExpansionNode(directionConfig, ACTIVE_CONFIG_DIST);
		// copy passive dofs
		expansionNode->getConfiguration()->copyPassive(*directionConfig);
		// expand the active dofs
		int nbCreatedNodes = this->ExpandProcess(expansionNode, directionConfig, directionNode, ENV.getExpansionMethod());
		// expand the passive dofs
		return(nbCreatedNodes + this->passiveExpandProcess(expansionNode, nbCreatedNodes, directionNode));
	}
	// Standard expansion case
	else
	{
		// get direction
		directionConfig = fromComp->getExpansionDirection(toComp, true, directionNode);
		// get node for expansion toward direction
		expansionNode = fromComp->getExpansionNode(directionConfig, ENV.getInt(Env::DistConfigChoice));
		// expansion
		return(this->ExpandProcess(expansionNode, directionConfig, directionNode,ENV.getExpansionMethod()));
	}
}

bool RRT::nextStep(LocalPath& path, Node* directionNode, double& pathDelta, shared_ptr<LocalPath>& newPath, Env::expansionMethod method)
{

	if(method == Env::Connect)
	{
		// create path satisfying connect method
		newPath = shared_ptr<LocalPath>(new LocalPath(path, pathDelta));
		if(pathDelta == 0.)
		{ return(false); }
	}
	else
	{
		pathDelta = path.Length() == 0. ? 1. : MIN(1., this->step() / path.Length());

		newPath = shared_ptr<LocalPath>(new LocalPath(path.getBegin(),
				path.configAtParam(_Robot, pathDelta * path.getLocalpathStruct()->range_param)));
	}

	return(newPath->getValid());
}

void RRT::expansionFailed(Node* node) {
	if(ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)  {
		p3d_SetNGood(0);
	}
	node->getNodeStruct()->n_fail_extend++;
	if(ENV.getBool(Env::discardNodes) &&
			(node->getNodeStruct() != _Graph->getGraphStruct()->search_start) &&
			(node->getNodeStruct() != _Graph->getGraphStruct()->search_goal) &&
			(node->getNodeStruct()->n_fail_extend > ENV.getInt(Env::MaxExpandNodeFail))) {
		node->getNodeStruct()->IsDiscarded = true;
		update_parent_nfails(node->getNodeStruct());
		_Graph->getGraphStruct()->n_consec_pb_level ++;
		_Graph->getGraphStruct()->n_consec_fail_pb_level ++;
	}
}

Node* RRT::connectNode(Node* currentNode, LocalPath& path, double pathDelta, Node* directionNode, double currentCost, int& nbCreatedNodes)
{
	if((pathDelta == 1. && directionNode))
	{
		_Graph->MergeComp(currentNode, directionNode, path.Length());
		return(directionNode);
	}
	else
	{
		Node* newNode = _Graph->insertNode(path.getEnd(), currentNode, currentCost, this->step());
		nbCreatedNodes++;
		return(newNode);
	}
}

void RRT::adjustTemperature(bool accepted, Node* node)
{
	double factor = exp(log(2.) / pow(10., ENV.getDouble(Env::temperatureRate)));
	if(accepted)
	{
		node->setTemp(node->getTemp() / 2.0);
		node->getCompcoStruct()->temperature /= 2.;
	}
	else
	{
		node->setTemp(node->getTemp() * factor);
		node->getCompcoStruct()->temperature *= factor;
	}
}

int RRT::ExpandProcess(Node* expansionNode,
		shared_ptr<Configuration> directionConfig,
		Node* directionNode,
		Env::expansionMethod method) {
	bool extensionSucceeded(false);
	bool failed(false);
	int nbCreatedNodes(0);
	Node* fromNode(expansionNode);
	Node* extensionNode(NULL);
	shared_ptr<LocalPath> directionLocalpath;
	double positionAlongDirection(0.);
	shared_ptr<LocalPath> extensionLocalpath;
	double extensionCost(0.);
	bool firstIteration(true);


	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without checking for expansion control
	while(firstIteration ||
			(method == Env::nExtend && !failed && positionAlongDirection < 1.))
	{
		directionLocalpath = shared_ptr<LocalPath>(new LocalPath(fromNode->getConfiguration(), directionConfig));
		extensionSucceeded = this->nextStep(*directionLocalpath, directionNode, positionAlongDirection, extensionLocalpath, method);
		failed |= !extensionSucceeded;
		// Transition test for cost spaces, increase temperature in case of failure
		if(!failed && ENV.getBool(Env::isCostSpace))
		{
			extensionCost = extensionLocalpath->getEnd()->cost();
		}
		// Expansion Control
		if(firstIteration && !failed)
		{
			if(ENV.getBool(Env::expandControl)  &&
					!this->expandControl(*directionLocalpath, positionAlongDirection, expansionNode))
				failed = true;
		}
		// Add node to graph if everything succeeded
		if(!failed)
		{
			extensionNode = this->connectNode(fromNode, *extensionLocalpath, positionAlongDirection, directionNode, extensionCost, nbCreatedNodes);
		}
		if(firstIteration && failed)
			this->expansionFailed(expansionNode);

		fromNode = extensionNode;
		firstIteration = false;
	}

	if(ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::CostMethodChoice) == MAXIMAL_THRESHOLD) {
		p3d_updateCostThreshold();
	}

	return nbCreatedNodes;
}

int RRT::passiveExpandProcess(Node* expansionNode,
		int NbActiveNodesCreated, Node* directionNode) {
	Node* lastCreatedNode;
	int nbPasExp(0);
	bool firstPass(true);
	bool reiterate(true);

	if((ENV.getBool(Env::isPasExtWhenAct)== FALSE)  && (NbActiveNodesCreated == 0)){
		/* The passive dof expansion only works if the
       active dofs have been expanded */
		return 0;
	}

	shared_ptr<Configuration> invalConf = shared_ptr<Configuration>(new Configuration(_Robot));
	/*Warning: I don't anderstand the function of the Reiterate parameter */
	vector<p3d_jnt*> oldJoints;
	//   while(reiterate && p3d_ExpanBlockedByColl(_Robot->get_robot(), invalConf->get_configPt()))
	//   {
	while(reiterate)
	{

		p3d_copy_config_into(_Robot->getRobotStruct(),_Robot->getRobotStruct()->currect_q_inv,invalConf->getConfigPtStruct());
		reiterate = false;
		vector<p3d_jnt*> joints;
		if(getCollidingPassiveJntList(_Robot->getRobotStruct(), invalConf->getConfigurationStruct(), joints))
		{
			vector<p3d_jnt*> newJoints;
			// select only the passive parameters that have not been expanded yet
			if(selectNewJntInList(_Robot->getRobotStruct(), joints, oldJoints, newJoints))
			{
				if((NbActiveNodesCreated == 0)&& firstPass) {
					/* No node has been created during the active node expansion */
					lastCreatedNode = expansionNode;
				} else {
					lastCreatedNode = _Graph->getNode(_Graph->getGraphStruct()->last_node->N);
				}
				firstPass = false;
				shared_ptr<Configuration> newRandConf = (_Graph->getLastnode()->getConfiguration()->copy());
				bool expansionSucceeded(false);
				for(int i(0); !expansionSucceeded; i++)
				{
					if(i >= ENV.getInt(Env::MaxPassiveExpand))
						return(nbPasExp);
					shoot_jnt_list_and_copy_into_conf(_Robot->getRobotStruct(), newRandConf->getConfigurationStruct(), newJoints);
					int nbCreatedNodes = this->ExpandProcess(lastCreatedNode, newRandConf, directionNode, ENV.getExpansionMethod());
					if(nbCreatedNodes > 0)
					{
						reiterate = true;
						expansionSucceeded = true;
						nbPasExp += nbCreatedNodes;
						if(ML_DEBUG)
							cout << "Expanded passive parameters at try " << i+1 << endl;
					}
				}
			}
		}
	}
	return(nbPasExp);
}

bool RRT::expandControl(LocalPath& path, double positionAlongDirection, Node* compNode)
{
	if(path.Length() <= this->step() &&
			positionAlongDirection >= 1.)
	{
		if(compNode->getCompcoStruct()->nbRefinNodes*2 > compNode->getCompcoStruct()->nnode)
			return(false);
		else
		{
			compNode->getCompcoStruct()->nbRefinNodes++;
		}
	}
	return(true);
}

bool RRT::manhattanSamplePassive() {
	return(ENV.getDouble(Env::manhatRatio) < p3d_random(0.,1.));
}

uint RRT::expand(p3d_graph* GraphPt, int (*fct_stop)(void), void (*fct_draw)(void))
{
	//   p3d_InitRun(_Graph->get_graph(), _Start->get_node(),
	// 	      _Goal ? _Goal->get_node() : NULL);

	if(ENV.getBool(Env::isCostSpace) &&
			(ENV.getExpansionMethod() == Env::Connect) )
	{
		cout << "Warning: Connect expansion strategy \
		is usually unadapted for cost spaces\n" << endl;
	}

	if((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal)) &&
			_Start->getConfiguration()->equal(*_Goal->getConfiguration()))
	{
		cout << "Tree Expansion failed: root nodes are the same" << endl;
		return(0);
	}

	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;
	Node* fromNode = _Start;
	Node* toNode = _Goal;

	if(ENV.getBool(Env::expandToGoal))
		_Start->connectNodeToCompco(_Goal,this->step());

	while(!this->checkStopConditions(*fct_stop))
	{
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if(! (ENV.getBool(Env::biDir) &&
				ENV.getBool(Env::expandBalanced) &&
				(fromNode->getCompcoStruct()->nnode >
	toNode->getCompcoStruct()->nnode + 2)))
		{
			NbCurCreatedNodes = (ENV.getBool(Env::biDir) ?
					this->expandOneStep(fromNode, toNode) :
						this->expandOneStep(_Start, _Goal));
			if(NbCurCreatedNodes != 0)
			{
				if (ENV.getBool(Env::drawGraph))
				{
					*GraphPt = *(_Graph->getGraphStruct());
					(*fct_draw)();
				}
				NbTotCreatedNodes += NbCurCreatedNodes;
				_nbConscutiveFailures = 0;
				if(ENV.getBool(Env::expandToGoal))
					if ((ENV.getBool(Env::biDir) ? toNode : _Goal)->connectNodeToCompco(_Graph->getLastnode(),this->step()))
					{printf("connected\n");}
			}
			else
				_nbConscutiveFailures++;
		}
		if(ENV.getBool(Env::biDir))
			swap(fromNode, toNode);
	}
	if(ENV.getBool(Env::drawGraph))
	{
		*GraphPt = *(_Graph->getGraphStruct());
		(*fct_draw)();
	}
	*GraphPt = *(_Graph->getGraphStruct());
	return(NbTotCreatedNodes);
}

