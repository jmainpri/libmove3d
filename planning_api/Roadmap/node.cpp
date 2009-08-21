//
// C++ Implementation: node
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "../planningAPI.hpp"

using namespace std;
using namespace tr1;

//Constructor and destructor
Node::Node(Graph* G, shared_ptr<Configuration> C)
{
  _Graph = G;
  _Robot = G->getRobot();
  _Configuration = C;
  _activ = false;
  _Node = p3d_APInode_make_multisol(G->getGraphStruct(), C->getConfigStruct(), NULL);
  p3d_create_compco(G->getGraphStruct(),_Node);
}

Node::Node(Graph* G, p3d_node* N)
{
  _Graph = G;
  _Robot = G->getRobot();
  _Configuration = shared_ptr<Configuration>(new Configuration(_Robot, N->q));
  _activ = false;
  _Node = N;
  if (_Node->comp == NULL)
    p3d_create_compco(G->getGraphStruct(),_Node);
}


Node::~Node()
{
  //p3d_APInode_desalloc(_Graph->get_graph(), _Node);
}

//Accessors
p3d_node* Node::getNodeStruct()
{
  return _Node;
}

Graph* Node::getGraph()
{
  return _Graph;
}

Robot* Node::getRobot()
{
  return _Robot;
}

shared_ptr<Configuration> Node::getConfiguration()
{
  return _Configuration;
}

void Node::activ(bool b)
{
  _activ = b;
}

bool Node::isActiv()
{
  return _activ;
}

void Node::setCompco(p3d_compco* compco)
{
  _Node->comp = compco;
}

p3d_compco* Node::getCompcoStruct()
{
  return(_Node->comp);
}

p3d_compco** Node::getCompcoStructPt()
{
  return(&(_Node->comp));
}

double Node::getCost()
{
  return(_Node->cost);
}

double Node::getDist()
{
  return (_Node->dist_Nnew);
}

double Node::dist(Node* N)
{
  double d = this->getConfiguration()->dist(*N->getConfiguration());
  _Node->dist_Nnew = d;
  return d;
}

bool Node::equal(Node* N)
{
  return (p3d_equal_config(_Robot->getRobotStruct(),N->getNodeStruct()->q, _Node->q));
}

bool Node::inSameComponent(Node* N)
{
  return(_Node->comp->num == N->getNodeStruct()->comp->num);
}

bool Node::isLinkable(Node* N, double* dist)
{
  return p3d_APInode_linked(_Graph->getGraphStruct(), _Node, N->getNodeStruct(), dist);
}

void Node::checkStopByWeight()
{
  double stopWeight;
  int signStopWeight;
  p3d_GetStopWeightAndSign(&stopWeight, &signStopWeight);
  if(signStopWeight * (_Node->weight - stopWeight) > 0)
  {
    p3d_SetStopValue(true);
    p3d_SetDiffuStoppedByWeight(true);
  }
}


//fonctions sur les composantes connexes
void Node::deleteCompco()
{
  p3d_remove_compco(_Graph->getGraphStruct(), _Node->comp);
}

bool Node::maximumNumberNodes()
{
  return this->getCompcoStruct()->nnode >= ENV.getInt(Env::maxNodeCompco);
}

bool Node::connectNodeToCompco(Node* N, double step)
{
  if(ENV.getBool(Env::isCostSpace))
    return (this->costConnectNodeToComp(N, step));
  else
    return (p3d_ConnectNodeToComp(N->getGraph()->getGraphStruct(), N->getNodeStruct(), _Node->comp));
}

bool Node::costConnectNodeToComp(Node* N, double step)
{
  int SavedIsMaxDis = FALSE;
  Node* node2(NULL);

  switch(p3d_GetNodeCompStrategy()) {
  case K_NEAREST_NODE_COMP:
    /*Connect randomly to one of the k nearest
      nodes of the componant */
    break;
  case NEAREST_NODE_COMP:
  default:
    SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
    p3d_SetIsMaxDistNeighbor(FALSE);
    node2 = _Graph->nearestWeightNeighbour(this,
					 N->getConfiguration(),
					 false,
					 p3d_GetDistConfigChoice());

    p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);


    shared_ptr<LocalPath> path = shared_ptr<LocalPath>(new LocalPath(N->getConfiguration(), node2->getConfiguration()));
    if(_Graph->linkNode(node2) && path->length() < step)
    {
      cout << "attempting connect " << N->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
 //     ((RRT*)plannerlist[0])->ExpandProcess(N, node2->getConfiguration(), node2, ENV.getExpansionMethod());
    }
    break;
  }
}

//place la compco dans la CompCo presente
void Node::merge(Node* compco)
{
  p3d_merge_comp(_Graph->getGraphStruct(), compco->getCompcoStruct(), &(_Node->comp));
}

bool Node::equalCompco(Node* compco)
{
  return (_Node->comp == compco->getCompcoStruct());
}

Node* Node::randomNodeFromComp()
{
  return(_Graph->getNode(p3d_RandomNodeFromComp(_Node->comp)));
}

Node* Node::getExpansionNode(shared_ptr<Configuration> direction, int distance)
{
  return (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH ?
	  _Graph->getNode(this->getCompcoStruct()->dist_nodes->N) :
	  this->selectExpantionNode(direction, distance));
}


shared_ptr<Configuration> Node::getExpansionDirection(Node* to_compco, bool samplePassive, Node*& direction_node)
{
  if(p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
  {
    shared_ptr<Configuration> q = _Graph->getRobot()->shoot(samplePassive);
    return(q->add(*(shared_ptr<Configuration>(new Configuration(_Graph->getRobot(),_Node->comp->dist_nodes->N->q)))));
  }
  else
  {
    shared_ptr<Configuration> q = this->selectExpansionDirection(to_compco, samplePassive, direction_node);
    return(q);
  }
}

Node* Node::selectExpantionNode(shared_ptr<Configuration> direction, int distance)
{
  int KNearest = -1;
  int NearestPercent = -1;

 switch(distance) {

 case NEAREST_EXP_NODE_METH:
   /* Choose the nearest node of the componant*/
   return(_Graph->nearestWeightNeighbour(this, direction, p3d_GetIsWeightedChoice(), distance));
 case K_NEAREST_EXP_NODE_METH:
   /* Select randomly among the K nearest nodes of a componant */
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(this->getCompcoStruct()->nnode))/100.));
   // TODO : fix
   //   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
   // KNearest);
   return(_Graph->nearestWeightNeighbour(this, direction, p3d_GetIsWeightedChoice(), distance));
 case BEST_SCORE_EXP_METH:
   /* Select the node which has the best score: weight*dist */
   return(_Graph->nearestWeightNeighbour(this, direction, p3d_GetIsWeightedChoice(), distance));
 case K_BEST_SCORE_EXP_METH:
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(this->getCompcoStruct()->nnode))/100.));
   // TODO : fix
   // ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
   return(_Graph->nearestWeightNeighbour(this, direction, p3d_GetIsWeightedChoice(), distance));
 case RANDOM_IN_SHELL_METH:
   /* Select randomly among all the nodes inside a given portion of shell */
   return(_Graph->getNode(hrm_selected_pb_node(_Graph->getGraphStruct(),
					   direction->getConfigStruct(),
					   this->getCompcoStruct())));
 case RANDOM_NODE_METH:
   return(_Graph->getNode(p3d_RandomNodeFromComp(this->getCompcoStruct())));
 default:
   /* By default return the nearest node of the componant */
   return(_Graph->nearestWeightNeighbour(this, direction, p3d_GetIsWeightedChoice(), distance));
  }
}


shared_ptr<Configuration> Node::selectExpansionDirection(Node* to_compco, bool samplePassive, Node*& direction_node)
{
  shared_ptr<Configuration> C;
  int savedRlg;

  if(!p3d_GetIsDirSampleWithRlg()) {
    // Save the previous Rlg setting to shoot without Rlg
    savedRlg = p3d_get_RLG();
    p3d_set_RLG(false);
  }

  // Selection in the entire CSpace and
  // biased to the Comp of the goal configuration
  if(p3d_GetIsGoalBias() &&
     p3d_random(0.,1.) <= p3d_GetGoalBiasValue())
  {
    // select randomly a node in the goal component as direction of expansion
    direction_node = to_compco->randomNodeFromComp();
    C = direction_node->getConfiguration()->copy();
  }
  else
  {
    switch(p3d_GetExpansionDirectionMethod())
    {
    case SUBREGION_CS_EXP:
      // Selection in a subregion of the CSpace
      // (typically close to the current tree)
      // and  biased to the goal configuration
      C = shared_ptr<Configuration>(new Configuration(_Graph->getRobot()));
     /* p3d_shoot_inside_box(_Graph->getRobot()->getRobotStruct(),
			   this->getCompcoStruct(),
			   C->getConfigurationStruct(),
			   this->getCompcoStruct()->box_env_small,
			   samplePassive);*/
      break;
    case GLOBAL_CS_EXP:
    default:
      // Selection in the entire CSpace
      C = _Graph->getRobot()->shoot(samplePassive);
    }
  }
  if(!p3d_GetIsDirSampleWithRlg())
  {
    //Restore the previous Rlg setting
    p3d_set_RLG(savedRlg);
  }
  return(C);
}


