//
// C++ Implementation: graph
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

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "cost_space.hpp"

#ifdef LIGHT_PLANNER
#include "../../lightPlanner/proto/lightPlannerApi.h"
#endif

using namespace std;
using namespace tr1;

// Constructors
//----------------------------------------------

Graph::Graph()
{
	
}

Graph::Graph(Robot* R, p3d_graph* G)
{
    if (G)
    {
        _Graph = MY_ALLOC(p3d_graph, 1);
        *_Graph = *G;
    }
    else
    {
        _Graph = p3d_create_graph();
    }
    _Robot = R;
    _Graph->rob->GRAPH = _Graph;
    _Traj = NULL;
    this->init();
}

Graph::Graph(Robot* R)
{
    _Robot = R;
    _Graph = p3d_allocinit_graph();

    _Graph->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
    _Graph->rob = _Robot->getRobotStruct();
    if (_Robot->getRobotStruct() != NULL)
    {
        _Robot->getRobotStruct()->GRAPH = _Graph;
        XYZ_GRAPH = _Graph;
    }
    if (STAT)
    {
        _Graph->stat = createStat();
    }
    else
    {
        _Graph->stat = NULL;
    }
    _Traj = NULL;
    this->init();
}

Graph::Graph(p3d_graph* G)
{
    if (G)
    {
        /*_Graph = MY_ALLOC(p3d_graph, 1);
                *_Graph = *G;*/
        _Graph = G;
    }
    else
    {
        _Graph = p3d_create_graph();
    }
    _Robot = new Robot(_Graph->rob);
    _Traj = NULL;
    this->init();
}

/**
  * Fonction creant un Objet Graph a partir d'une structure de p3d_Graph
  */
void Graph::init()
{
    if (_Graph->nodes)
    {
        p3d_list_node* l = _Graph->nodes;
        while (l)
        {
            Node* node = new Node(this, l->N);
            _NodesTable.insert(pair<p3d_node*, Node*> (l->N, node));
            _Nodes.push_back(node);
            l = l->next;
        }
    }
    if (_Graph->edges)
    {
        p3d_list_edge* l = _Graph->edges;
        while (l)
        {
            Edge* edge = new Edge(this, l->E);
            _Edges.push_back(edge);
            l = l->next;
        }
    }
    this->setName();
}

// Destructors
//----------------------------------------------

Graph::~Graph()
{
    p3d_del_graph(_Graph);
    this->freeResources();
}

/**
  * Frees the Nodes and Edges
  */
void Graph::freeResources()
{

    for(unsigned i=0;i<_Nodes.size();i++)
    {
        delete _Nodes[i];
    }

    for(unsigned i=0;i<_Edges.size();i++)
    {
        delete _Edges[i];
    }

}

// Import and Export function to p3d
//----------------------------------------------
/**
 * Fonction that imports a Graph
 */
/*void Graph::importGraphStruct(p3d_graph* G)
{
	cout << "Importing the graph" << endl;
	
    if (G->nodes)
    {
        p3d_list_node* l = G->nodes;
        while (l)
        {
            _Nodes.push_back(new Node(this, l->N));
            l = l->next;
        }
    }
    if (G->edges)
    {
        p3d_list_edge* l = G->edges;
        while (l)
        {
            _Edges.push_back(new Edge(this, 
									  static_cast<unsigned int>(l->E->Ni->num - 1 ), 
									  static_cast<unsigned int>(l->E->Nf->num - 1 ) ));
            l = l->next;
        }
    }
	if (G->comp)
    {
        p3d_compco* l = G->comp;
        while (l)
        {
            m_Comp.push_back(new ConnectedComponent(this, l));
            l = l->suiv;
        }
    }
	
    this->setName();
}*/

/**
 * Fonction that imports a Graph
 */
/*p3d_graph* Graph::exportGraphStruct()
{
    p3d_graph* G = p3d_allocinit_graph();
	
    G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
    G->rob = _Robot->getRobotStruct();
	
    _Robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(_Nodes.size());
	G->nedge = static_cast<int>(_Edges.size());
	
    if (STAT)
    {
        G->stat = createStat();
    }
    else
    {
        G->stat = NULL;
    }
	
	cout << "m_Nodes.size() = " << _Nodes.size() << endl;
	
	p3d_list_node* ln = new p3d_list_node;
	p3d_list_node* prevNode;
	ln->prev = NULL;
	G->nodes = ln;
	for (unsigned int i = 0; i < _Nodes.size(); i++) 
	{
		ln->N = _Nodes[i]->getNodeStruct();
		
		if ( i < _Nodes.size() - 1 ) 
		{
			prevNode = ln;
			prevNode->next = new p3d_list_node;
			ln = prevNode->next;
			ln->prev = prevNode;
		}
		else 
		{
			ln->next = NULL;
		}
		
		//cout << "New Node in list" << i << endl;
	}
	G->last_node = ln;
	
	
	cout << "m_Edges.size() = " << _Edges.size() << endl;
	
	p3d_list_edge* le = new p3d_list_edge;
	p3d_list_edge* prevEdge;
	le->prev = NULL;
	G->edges = le;
	for (unsigned int i = 0; i < _Edges.size(); i++) 
	{
		//m_Edges.push_back(new Edge(this, l->E));
		le->E = _Edges[i]->getEdgeStruct();
		
		if ( i < _Edges.size() - 1 ) 
		{
			prevEdge = le;
			prevEdge->next = new p3d_list_edge;
			le = prevEdge->next;
			le->prev = prevEdge;
		}
		else 
		{
			le->next = NULL;
		}
		
		//cout << "New Edges in list" << i << endl;
	}
	G->last_edge = le;
	
	
	p3d_compco* lc;
	p3d_compco* prevComp;
	lc->prec = NULL;
	lc = m_Comp[0]->getCompcoStruct();
	G->comp = lc;
	for (unsigned int i = 1; i < m_Comp.size(); i++) 
	{
		prevComp = lc;
		lc = m_Comp[i]->getCompcoStruct();
		prevComp->suiv = lc;
		lc->prec = prevComp;
	}
	lc->suiv = NULL;
	//G->last_edge = lc;
	
	return G;
}*/


// Accessors
//----------------------------------------------
p3d_graph* Graph::getGraphStruct()
{
    return _Graph;
}

void Graph::setGraph(p3d_graph* G)
{
    *_Graph = *G;
}

Robot* Graph::getRobot()
{
    return _Robot;
}

/*void Graph::setRobot(Robot* R)
{
	_Robot = R;
}*/

void Graph::setTraj(p3d_traj* Traj)
{
    _Traj = Traj;
}

p3d_traj* Graph::getTrajStruct()
{
    return _Traj;
}

vector<Node*> Graph::getNodes()
{
    return _Nodes;
}

vector<Edge*> Graph::getEdges()
{
    return _Edges;
}

map<p3d_node*, Node*> Graph::getNodesTable()
{
    return _NodesTable;
}

Node* Graph::getNode(p3d_node* N)
{
    map<p3d_node*, Node*>::iterator it = _NodesTable.find(N);
    if (it != _NodesTable.end())
        return (it->second);
    else
    {
        cout << "node " << N << " not found: num: " << N->num << endl;
        return (NULL);
    }
}

Node* Graph::getLastnode()
{
    return (_Nodes.back());
}

string Graph::getName()
{
    return _Name;
}

void Graph::setName()
{
    _Name = "graph";
}

void Graph::setName(string Name)
{
    _Name = Name;
}

bool Graph::equal(Graph* G)
{
    return (_Graph->nodes == G->getGraphStruct()->nodes);
}

bool Graph::equalName(Graph* G)
{
    return (_Name == G->getName());
}

// Graph operations
//----------------------------------------------
/**
 * Is Edge in graph
 */
bool Graph::isEdgeInGraph(Node* N1, Node* N2)
{
    bool flag = false;
	
    for (uint i = 0; i < _Edges.size(); i = i + 1)
    {
        flag =
		( _Edges[i]->getEdgeStruct()->Ni == N1->getNodeStruct() ) &&
		( _Edges[i]->getEdgeStruct()->Nf == N2->getNodeStruct() );
        if (flag)
        {
            return flag;
        }
    }
    return flag;
}

/**
  * Search configuration in graph
  */
Node* Graph::searchConf(Configuration& q)
{
    p3d_node* node(p3d_TestConfInGraph(_Graph,q.getConfigStruct()));
    return (node ? _NodesTable[node] : NULL);
}

/**
 * Create a Compco
 */
void Graph::createCompco(Node* node)
{
	p3d_create_compco(_Graph, node->getNodeStruct() );
}

/**
  * Insert node in graph
  */
Node* Graph::insertNode(Node* node)
{
    _NodesTable.insert(pair<p3d_node*, Node*> (node->getNodeStruct(), node));
    _Nodes.push_back(node);

    p3d_insert_node(_Graph, node->getNodeStruct());

    this->getGraphStruct()->dist_nodes = p3d_add_node_to_list(
            node->getNodeStruct(),
            this->getGraphStruct()->dist_nodes);

    return (node);
}

/**
  * Insert Extremal Node in Graph
  */
Node* Graph::insertExtremalNode(Node* node)
{
    this->insertNode(node);
    node->getNodeStruct()->type = ISOLATED;
    return (node);
}

/**
 * Insert node for RRT
 */
Node* Graph::insertNode(Node* expansionNode,LocalPath& path)

/*shared_ptr<Configuration> q,
 Node* expansionNode,
 double currentCost, double step)*/
{
    double step = path.getParamMax();
	
	
    Node* node(this->insertConfigurationAsNode(path.getEnd(), expansionNode, step));
	
    // Cost updates
    if (ENV.getBool(Env::isCostSpace))
    {
        double currentCost = path.getEnd()->cost();
		
        p3d_SetNodeCost(_Graph, node->getNodeStruct(), currentCost );
		
        //for adaptive variant, new temp is refreshed except if it is going down.
        if (currentCost < expansionNode->getNodeStruct()->cost)
        {
            node->getNodeStruct()->temp = expansionNode->getNodeStruct()->temp;
        }
        else
        {
            node->getNodeStruct()->temp = expansionNode->getNodeStruct()->temp
			/ 2.;
        }
    }
	
    //weight updates
    if (p3d_GetIsWeightedChoice())
        p3d_SetNodeWeight(_Graph, node->getNodeStruct());
	
    //check stop conditions
    if (p3d_GetIsWeightStopCondition())
    {
        node->checkStopByWeight();
    }
	
    // Graph updates for RANDOM_IN_SHELL method
    if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)
    {
        p3d_SetNGood(p3d_GetNGood() + 1);
		
        if (node->getNodeStruct()->weight > _Graph->CurPbLevel)
        {
            _Graph->CurPbLevel = node->getNodeStruct()->weight;
            _Graph->n_consec_pb_level = 0;
            _Graph->n_consec_fail_pb_level = 0;
            if (p3d_GetNGood() > 2)
                _Graph->critic_cur_pb_level = _Graph->CurPbLevel;
        }
        else
        {
            _Graph->n_consec_pb_level++;
            _Graph->n_consec_fail_pb_level = 0;
        }
    }
	
    //Additional cycles through edges addition if the flag is active
    if (ENV.getBool(Env::addCycles))
    {
        this->addCycles(node, step);
    }
	
	
    return (node);
}

/**
 * Insert Lining Node for RRT
 */
Node* Graph::insertConfigurationAsNode(shared_ptr<Configuration> q, Node* from,double step)
{
    Node* node = new Node(this, q);
	
    this->insertNode(node);
	
    if (node->getCompcoStruct()->num < from->getCompcoStruct()->num)
    {
        p3d_merge_comp(_Graph,
                       node->getCompcoStruct(),
                       from->getCompcoStructPt());
    }
    else if (from->getCompcoStruct()->num < node->getCompcoStruct()->num)
    {
        p3d_merge_comp(_Graph,
                       from->getCompcoStruct(),
                       node->getCompcoStructPt());
    }
	
    p3d_create_edges(_Graph,
                     from->getNodeStruct(),
                     node->getNodeStruct(),
                     step);
	
    node->getNodeStruct()->rankFromRoot = from->getNodeStruct()->rankFromRoot +1;
    node->getNodeStruct()->type = LINKING;
	
    return(node);
}

/**
  * Compare Length Edges
  */
bool Graph::compareEdges(Edge* E1, Edge* E2)
{
    return (E1->getEdgeStruct()->longueur < E2->getEdgeStruct()->longueur);
}

/**
  * Compare Nodes
  */
bool Graph::compareNodes(Node* N1, Node* N2)
{
    return (N1->getNodeStruct()->dist_Nnew < N2->getNodeStruct()->dist_Nnew);
	//double dist = N1->dist(N2);
	//return dist;
}

/**
  * Sort Edges
  */
void Graph::sortEdges()
{
    if (_Edges.size() > 0)
    {
        sort(_Edges.begin(), _Edges.end(), &compareEdges);
    }
}

/**
  * Sort Node by distance
  */
void Graph::sortNodesByDist(Node* N)
{
    if (_Nodes.size() > 1)
    {
        for (unsigned int i=0; i < _Nodes.size(); i++)
        {
            _Nodes[i]->dist(N);
        }
        sort(_Nodes.begin(), _Nodes.end(), &compareNodes);
    }
}

/**
  * Add Edge to Graph
  */
void Graph::addEdge(Node* N1, Node* N2, double Long)
{
    if (!this->isEdgeInGraph(N1, N2))
    {
        Edge* E = new Edge(this, N1, N2, Long);
        _Edges.push_back(E);
    }
}

/**
  * Add Edges
  * WARNING only in graph API doesn't work yet not to be used
  */
void Graph::addEdges(Node* N1, Node* N2, double Long)
{
    this->addEdge(N1, N2, Long);
    this->addEdge(N2, N1, Long);
}

/**
  * Add Edge to all node inferior to max Dist form N
  */
void Graph::addNode(Node* N, double maxDist)
{
    double d;
    for (uint i = 0; i < _Nodes.size(); i = i + 1)
    {
        if ((d = _Nodes[i]->getConfiguration()->dist(*N->getConfiguration()))
            < maxDist)
            {
            LocalPath path(_Nodes[i]->getConfiguration(), N->getConfiguration());

            if ( path.isValid() )
            {
                this->addEdges(_Nodes[i], N, d);
            }
        }
    }
    _Nodes.push_back(N);
}

/**
  * Add  vector of node
  */
void Graph::addNodes(vector<Node*> N, double maxDist)
{
    for (vector<Node*>::iterator it = N.begin(), end = N.end(); it != end; it++)
    {
        this->addNode(*it, maxDist);
    }
}

/**
  * Test if node is in graph
  */
bool Graph::isInGraph(Node* N)
{
    for (uint i = 0; i < _Nodes.size(); i = i + 1)
    {
        if (N->equal(_Nodes[i]))
        {
            return true;
        }
    }
    return false;
}

/**
  * Link node to graph
  */
bool Graph::linkNode(Node* N)
{
    if (ENV.getBool((Env::useDist)))
    {
        return this->linkNodeAtDist(N);
    }
    else
    {
        return this->linkNodeWithoutDist(N);
    }
}

/**
  * Links node to graph Without distance
  */
bool Graph::linkNodeWithoutDist(Node* N)
{
    bool b = false;
    for (int j = 1; j <= _Graph->ncomp; j = j + 1)
    {
        if (N->getNodeStruct()->numcomp != j)
        {
            for (uint i = 0; i < _Nodes.size(); i = i + 1)
            {
                if (_Nodes[i]->getNodeStruct()->numcomp == j)
                {
                    if (_Nodes[i]->connectNodeToCompco(N, 0))
                    {
                        N->getNodeStruct()->search_to
                                = N->getNodeStruct()->last_neighb->N;
                        N->getNodeStruct()->last_neighb->N->search_from
                                = N->getNodeStruct();
                        N->getNodeStruct()->last_neighb->N->edge_from
                                = N->getNodeStruct()->last_neighb->N->last_edge->E;
                        b = true;
                    }
                    break;
                }
            }
        }
    }
    return b;
}

/**
  * Links node at distance
  */
bool Graph::linkNodeAtDist(Node* N)
{
  int nbLinkedComponents = p3d_link_node_graph_multisol(N->getNodeStruct(), _Graph);
  this->mergeCheck();
  return(nbLinkedComponents > 0);
}

/**
  * Links Node to all nodes
  */
bool Graph::linkToAllNodes(Node* newN)
{
    return p3d_all_link_node(newN->getNodeStruct(), _Graph);
}

/**
  * Create Random Configurations
  */
void Graph::createRandConfs(int NMAX)
{
    int inode;
    double tu, ts;

    ChronoOn();

    inode = 0;

    shared_ptr<Configuration> Cs = shared_ptr<Configuration> (
            new Configuration(_Robot, _Robot->getRobotStruct()->ROBOT_POS));
    Node* Ns = new Node(this, Cs);
    this->insertNode(Ns);

    while (inode < NMAX)
    {
        shared_ptr<Configuration> C = _Robot->shoot();
        if (!C->IsInCollision())
        {
            this->insertNode(new Node(this, C));
            inode = inode + 1;
            if (fct_draw)
                (*fct_draw)();
            else
            {
                PrintInfo(("Random conf. generation in not possible\n"));
                break;
            }

            if (fct_stop)
            {
                if (!(*fct_stop)())
                {
                    PrintInfo(("Random confs. generation canceled\n"));
                    break;
                }
            }
        }
    }

    PrintInfo(("For the generation of %d configurations : ", inode));
    ChronoTimes(&tu, &ts);
    _Graph->time = _Graph->time + tu;
    ChronoPrint("");
    ChronoOff();
    p3d_print_info_graph(_Graph);
}

/**
  * Random Nodes From Component
  */
Node* Graph::randomNodeFromComp(Node* comp)
{
    return (this->getNode(p3d_RandomNodeFromComp(comp->getCompcoStruct())));
}

/**
  * Nearest Weighted Neighbout in graph
  */
Node* Graph::nearestWeightNeighbour(Node* compco, shared_ptr<Configuration> config,
                                    bool weighted, int distConfigChoice)
{
    p3d_node* BestNodePt = NULL;
    double BestScore = P3D_HUGE;
    double CurrentDist = -1.;
    double CurrentScore = -1.;
    double DistOfBestNode = -1.;
    p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;
    p3d_matrix4 MobFrameRef, invT;

    // When retrieving statistics
    if (getStatStatus())
    {
        _Graph->stat->planNeigCall++;
    }

    //computation of the mobFrameRef of the Config
    if (distConfigChoice == MOBILE_FRAME_DIST && p3d_GetRefAndMobFrames(
            _Graph->rob, &RefFramePt, &MobFramePt))
    {
        p3d_set_robot_config(_Graph->rob, config->getConfigStruct());
        p3d_update_this_robot_pos_without_cntrt_and_obj(_Graph->rob);
        p3d_GetRefAndMobFrames(_Graph->rob, &RefFramePt, &MobFramePt);
        if (RefFramePt == NULL)
        {
            p3d_mat4Copy(*MobFramePt, MobFrameRef);
        }
        else
        {
            p3d_matInvertXform(*RefFramePt, invT);
            p3d_matMultXform(invT, *MobFramePt, MobFrameRef);
        }
    }

#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::FKDistance))
    {

        if( ! config->setConstraints() )
        {
            cout << "Graph CPP API :: FK Constraint could not be satistfied"  << endl;
        }

        activateCcCntrts(_Robot->getRobotStruct(),-1,true);
    }
#endif

//    cout << "distConfigChoice = " << distConfigChoice << endl;

    p3d_list_node* nodes(compco->getCompcoStruct()->dist_nodes);
//    cout << "nearestWeightNeighbour "  << endl;
    while (nodes)
    {
        /* We take into account only the nodes undiscarded */
        if (!nodes->N->IsDiscarded)
        {
            if (distConfigChoice == MOBILE_FRAME_DIST)
            {
                CurrentDist = p3d_GetSe3DistanceFrames(_Graph->rob,
                                                       MobFrameRef, nodes->N->RelMobFrame);
            }
            else
            {

                CurrentDist = config->dist(
                        *_NodesTable[nodes->N]->getConfiguration(),
                        distConfigChoice);
            }

            CurrentScore = CurrentDist
                           * (weighted ? p3d_GetNodeWeight(nodes->N) : 1.0);

            if (CurrentScore < BestScore)
            {
                BestScore = CurrentScore;
                BestNodePt = nodes->N;
                DistOfBestNode = CurrentDist;
            }
        }
        nodes = nodes->next;
    }

#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::FKDistance))
    {
        deactivateCcCntrts(_Robot->getRobotStruct(),-1);
    }
#endif

    if ((p3d_GetIsMaxDistNeighbor() == TRUE) && (BestNodePt->boundary == TRUE)
        && (BestNodePt->radius < DistOfBestNode))
        {
        /* There is a maximal distance allowed to get a node as neighbor */
        return NULL;
        }

    return _NodesTable[BestNodePt];
}

/**
  * Merge Component
  */
int Graph::mergeComp(Node* CompCo1, Node* CompCo2, double DistNodes)
{

    if ((CompCo1 == NULL) || (CompCo2 == NULL))
    {
        PrintInfo (("Warning: Try to link two nodes with NULL structures \n"));
        return FALSE;
    }

    if (CompCo1->getCompcoStruct()->num < CompCo2->getCompcoStruct()->num)
    {
        CompCo1->merge(CompCo2);
    }
    else if (CompCo1->getCompcoStruct()->num > CompCo2->getCompcoStruct()->num)
    {
        CompCo2->merge(CompCo1);
    }

    this->addEdges(CompCo1, CompCo2, DistNodes);

    p3d_create_edges(_Graph,
                     CompCo1->getNodeStruct(),
                     CompCo2->getNodeStruct(),
                     DistNodes);

    return true;
}


/**
  * Get Nodes in the same compco
  */
std::vector<Node*> Graph::getNodesInTheCompCo(Node* node)
{
    p3d_list_node* ListNode = node->getCompcoStruct()->dist_nodes;
    std::vector<Node*> Nodes;

    while (ListNode!=NULL)
    {
        Nodes.push_back(this->_NodesTable[ListNode->N]);
        ListNode = ListNode->next;
    }
    return Nodes;
}


/**
  * Detect the need of merging comp
  */
void Graph::mergeCheck()
{
    p3d_merge_check(_Graph);
}

/**
  * Add Cycle in Graph
  */
void Graph::addCycles(Node* node, double step)
{
    double longStep = 3. * step;
    p3d_list_node* listDistNodePt = p3d_listNodeInDist(
            _Robot->getRobotStruct(), node->getNodeStruct()->comp,
            node->getNodeStruct(), longStep);

    p3d_list_node* savedListDistNodePt = listDistNodePt;

    shared_ptr<LocalPath> LP;

    while (listDistNodePt)
    {
        if (!p3d_IsSmallDistInGraph(_Graph, node->getNodeStruct(),
                                    listDistNodePt->N, 5, step))
        {
            LP = shared_ptr<LocalPath> (new LocalPath(node->getConfiguration(),
                                                      this->getNode(listDistNodePt->N)->getConfiguration()));
            if (LP->isValid()
                /*&& this->getNode(listDistNodePt->N)->getConfiguration()->costTestSucceeded(
                                                        node, step)
                                        && node->getConfiguration()->costTestSucceeded(
                                                        this->getNode(listDistNodePt->N), step)*/)

                {
                p3d_create_edges(_Graph, node->getNodeStruct(),
                                 listDistNodePt->N, LP->length());
                node->getNodeStruct()->edges->E->for_cycle = true;
            }
        }
        listDistNodePt = listDistNodePt->next;
    }
    while (savedListDistNodePt)
    {
        p3d_list_node* destroyListNodePt = savedListDistNodePt;
        savedListDistNodePt = savedListDistNodePt->next;
        MY_FREE(destroyListNodePt, p3d_list_node, 1);
    }
}

/**
 * Recompute all node and edge
 * cost
 */
void Graph::recomputeCost()
{
    for(unsigned int i=0; i<_Nodes.size();i++)
	{
		_Nodes[i]->getCost();
	}
	
	for(unsigned int i=0;i<_Edges.size();i++)
	{
		_Edges[i]->getEdgeCost();
	}
}

/**
 * Check if all edges are valid
 */
/*bool Graph::checkAllEdgesValid()
{
	int collTest = 0;
	for(unsigned int i=0;i<_Edges.size();i++)
	{
		shared_ptr<LocalPath> ptrLP = _Edges[i]->getLocalPath();
		
		if(!(ptrLP->isValid()))
		{
			return false;
		}
		
		collTest += ptrLP->getNbColTest();
	}
	
	cout << "Graph Total Number of coll. test = " << collTest << endl;
	
	return true;
}*/

void Graph::extractBestTraj(shared_ptr<Configuration> qi,shared_ptr<Configuration> qf)
{
	Node  *Ns=NULL,*Ng=NULL;

	//  p3d_graph* graphPt = NULL;
	double    tu;//,ts;
	p3d_traj* trajPt = NULL;
	//  ChronoOn();
	
	
	if(_Graph == NULL) {
		cout << "Warning: cannot extract the best path\
				   as there is no current graph" << endl;
		return;
	}
	// Dense Roadmap creation
	//  graphPt = XYZ_GRAPH;
	
	// start and goal config creation
//	qi = _Robot->getInitialPosition();
//	qf = _Robot->getGoTo();
	if(qf->IsInCollision()) {
		(_Graph->nb_test_coll)++;
		cout << "Computation of approximated optimal cost stopped: \
				   Goal configuration in collision" << endl;
		ChronoOff();
		return;
	}
	
	// start and goal nodes creation and initialisation
	Ns = searchConf(*qi);
	if(Ns == NULL) {
		Ns = new Node(this,qi);
		insertExtremalNode(Ns);
	} else {
		//p3d_destroy_config(robotPt, configStart);
		//configStart = NULL;
	}
	
	Ng = searchConf(*qf);
	if(Ng == NULL) {
		Ng = new Node(this,qf);
		insertExtremalNode(Ng);
	} else {
		//p3d_destroy_config(robotPt, configGoal);
		//configGoal = NULL;
	}
	initMotionPlanning(Ns,Ng);
	_Robot->setAndUpdate(*Ns->getConfiguration());
	
	//search of the main constructed connected compoant
	p3d_compco* actualCompco = _Graph->comp;
	p3d_compco* mainCompPt = actualCompco;
	for(int i=0;i<_Graph->ncomp;i++)
	{
		if (actualCompco->nnode > mainCompPt->nnode) {
			mainCompPt = actualCompco;
		}
		actualCompco = actualCompco->suiv;
	}
	
	//connection of the the extremal nodes to the main componant
	bool ConnectRes = p3d_ConnectNodeToComp(_Graph,  Ns->getNodeStruct(), mainCompPt);
	ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(_Graph, Ng->getNodeStruct(), mainCompPt)) ;
	
	if(!ConnectRes) 
	{
		cout << "The extremal nodes can't be connected" << endl;
	}
	else 
	{
		// on construit la trajectoire entre les points etapes
		(_Graph->nb_test_coll)++;
		cout << "Connection of the extremal nodes succeeded\n" << endl;
		cout << _Graph->ncomp << endl;
		trajPt = p3d_graph_to_traj(_Robot->getRobotStruct());
	}
	
	//time info
	// ChronoPrint("");
	// ChronoTimes(&tu,&ts);
	_Graph->time = _Graph->time + tu;
	// ChronoOff();
	
	//PrintInfo(("ConnectRes: %d\n",ConnectRes));
	if(ConnectRes == TRUE && trajPt) {
		Trajectory traj(_Robot,trajPt);
		cout << "Trajectory cost  = " << traj.cost() << endl;
	}
}

/**
 * Replaces p3d_InitRun
 */
void Graph::initMotionPlanning(Node* start,Node* goal)
{
#ifdef ENERGY
	int n_coldeg,icoldeg;
	double *coldeg_qs;
#endif
	_Graph->search_start = start->getNodeStruct();
	if(ENV.getBool(Env::expandToGoal)== true) 
	{
		_Graph->search_goal = goal->getNodeStruct();
	}
	
	_Graph->search_done = FALSE;
	p3d_InitDynDomainParam(_Graph,start->getNodeStruct(),goal->getNodeStruct());
	
	if(ENV.getBool(Env::isCostSpace) == true) {
		global_costSpace->initMotionPlanning(this, start, goal);
	}
	if (p3d_GetIsWeightedChoice()== TRUE) {
		p3d_init_root_weight(_Graph);
	}
	if(ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH ) {
		p3d_init_pb_variables(_Graph);
	}
	if(start != NULL) {
		start->getNodeStruct()->rankFromRoot = 1;
		start->getNodeStruct()->comp->nbTests = 0;
	}
	if(goal != NULL) {
		goal->getNodeStruct()->rankFromRoot = 1;
		goal->getNodeStruct()->comp->nbTests = 0;
	}
#ifdef ENERGY
	if(p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) {
		n_coldeg = bio_get_num_collective_degrees();
		// init coldeg_q in Ns
		coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
		for(icoldeg=0; icoldeg<n_coldeg; icoldeg++) {
			coldeg_qs[icoldeg] = 0.0;
		}
		bio_copy_coldeg_q_in_N(start,coldeg_qs,n_coldeg);
		bio_destroy_coldeg_config(coldeg_qs,n_coldeg);
		// WARNING : currently Ng is not considered !!!
	}
#endif
}