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

Node::Node():
	_SelectCost(0.0),
	_nbExpan(0)
{

}
//Constructor and destructor
Node::Node(const Node& N) :
	_Graph(N._Graph),
	_Robot(N._Robot),
	_Configuration(N._Configuration),
	_activ(false),
	_SelectCost(0.0),
	_nbExpan(0)
{
	_Node = N._Node;
}

//Constructor and destructor
Node::Node(Graph* G, shared_ptr<Configuration> C) :
		_SelectCost(0.0),
		_nbExpan(0)
{
	_Graph = G;
	_Robot = G->getRobot();
	_Configuration = C;
	_activ = false;
	_Node = p3d_APInode_make_multisol(G->getGraphStruct(),
			C->getConfigStruct(), NULL);
	p3d_create_compco(G->getGraphStruct(), _Node);
}

Node::Node(Graph* G, p3d_node* N) :
	_SelectCost(0.0),
	_nbExpan(0)
{
	_Graph = G;
	_Robot = G->getRobot();
	_Configuration
			= shared_ptr<Configuration> (new Configuration(_Robot, N->q));
	_activ = false;
	_Node = N;
	if (_Node->comp == NULL)
		p3d_create_compco(G->getGraphStruct(), _Node);
}

bool Node::operator==(Node& N)
{
	return this->_Configuration->equal(*(N._Configuration.get()));
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
	return (_Node->comp);
}

p3d_compco** Node::getCompcoStructPt()
{
	return (&(_Node->comp));
}

double Node::getCost()
{
	_Node->cost = _Configuration->cost();
	return (_Node->cost);
}


double Node::getSumCost()
{
//	_Node->cost = _Configuration->cost();
	return (_Node->sumCost);
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
	return (p3d_equal_config(_Robot->getRobotStruct(), N->getNodeStruct()->q,
			_Node->q));
}

bool Node::inSameComponent(Node* N)
{
	return (_Node->comp->num == N->getNodeStruct()->comp->num);
}

std::vector<p3d_node*> Node::getNeighbors()
{
	vector<p3d_node*> allNeighbors;
	p3d_list_node* list = _Node->neighb;

	for(unsigned int i=0;i<_Node->nneighb;i++)
	{
		p3d_node* ptrNode = list->N;
		allNeighbors.push_back(ptrNode);
		list = list->next;
	}

	return allNeighbors;
}

std::vector<p3d_edge*> Node::getEdges()
{
	vector<p3d_edge*> allEdges;
	p3d_list_edge* list = _Node->edges;

	for(unsigned int i=0;i<_Node->nedge;i++)
	{
		p3d_edge* ptrEdge = list->E;
		allEdges.push_back(ptrEdge);
		list = list->next;
	}

	return allEdges;
}

bool Node::isLinkable(Node* N, double* dist)
{
	return p3d_APInode_linked(_Graph->getGraphStruct(), _Node,
			N->getNodeStruct(), dist);
}

void Node::checkStopByWeight()
{
	double stopWeight;
	int signStopWeight;
	p3d_GetStopWeightAndSign(&stopWeight, &signStopWeight);
	if (signStopWeight * (_Node->weight - stopWeight) > 0)
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
	if (ENV.getBool(Env::isCostSpace))
	{
		cout << "WARNING: Using Cost Space with wrong algortihm" << endl;
		return false;
	}
	else
	{
		return (p3d_ConnectNodeToComp(N->getGraph()->getGraphStruct(),
				N->getNodeStruct(), _Node->comp));
	}
}

//place la compco dans la CompCo presente
void Node::merge(Node* compco)
{
	p3d_merge_comp(_Graph->getGraphStruct(), compco->getCompcoStruct(),
			&(_Node->comp));
}

bool Node::equalCompco(Node* compco)
{
	return (_Node->comp == compco->getCompcoStruct());
}

Node* Node::randomNodeFromComp()
{
	return (_Graph->getNode(p3d_RandomNodeFromComp(_Node->comp)));
}

void Node::print()
{
	_Configuration->print();
}

