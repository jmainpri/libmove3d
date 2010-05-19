//
// C++ Implementation: acr
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ACR.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

ACR::ACR(Robot* R, Graph* G)
        : PRM(R,G)
{
	cout << "ACR::ACR() with robot : " << R->getName() << endl;
	cout << "Max connecting nodes : " << ENV.getInt(Env::maxConnect) << endl;
}

ACR::~ACR()
{
	
}


void ACR::expandOneStep()
{
	//p3d_add_all_prm_node(_Graph->getGraphStruct(), _stop_func );

	//N = p3d_APInode_shoot(_Graph->getGraphStruct());
	
	shared_ptr<Configuration> q = _Robot->shoot();
		
	if ( q->setConstraintsWithSideEffect() && (!q->IsInCollision()))
	{
		//Node* N = new Node();
		p3d_node *N = NULL;
		
		N = p3d_create_node(_Graph->getGraphStruct());
		N->q = p3d_alloc_config(_Robot->getRobotStruct());
		
		p3d_copy_config_into(_Robot->getRobotStruct(),
							 q->getConfigStruct(),
							 &(N->q));
		
		m_nbConscutiveFailures = 0;
	
		p3d_insert_node(_Graph->getGraphStruct(), N);
		p3d_all_link_node(N, _Graph->getGraphStruct());
	
		if (N->numcomp == -1) 
		{
			/* Node have not been included in a compco, create one for it */
			p3d_create_compco(_Graph->getGraphStruct(), N);
			p3d_merge_check(_Graph->getGraphStruct());
		}
	}
	
//	shared_ptr<Configuration> q = _Robot->shoot();
//	
//	if ( q->setConstraintsWithSideEffect() && (!q->IsInCollision()))
//	{
//		m_nbConscutiveFailures = 0;
//		
//		Node* N = new Node(_Graph,q);
//		
//		_Graph->insertNode(N);
//		_Graph->linkToAllNodes(N);
//		_Graph->MergeCheck();
		
		m_nbAddedNode ++;
		
		if (ENV.getBool(Env::drawGraph))
		{
			(*_draw_func)();
		}
//	}
//	else 
//	{
//		m_nbConscutiveFailures ++;
//	}

}

/*uint ACR::expand(p3d_graph* Graph_Pt,int (*fct_stop)(void), void (*fct_draw)(void))
{
    if(ENV.getBool(Env::expandToGoal) &&
       _Start->getConfiguration()->equal(*_Goal->getConfiguration()))
    {
        cout << "graph creation failed: start and goal are the same" << endl;
        return(0);
    }

    int nbAddedNode = 0;

    while(!this->checkStopConditions())
    {
        shared_ptr<Configuration> C = _Robot->shoot();
        _nbConscutiveFailures ++;
        if (!C->IsInCollision())
        {
            _nbConscutiveFailures = 0;
            Node* N = new Node(_Graph,C);
            _Graph->insertNode(N);
            _Graph->linkToAllNodes(N);

            nbAddedNode ++;
            if (ENV.getBool(Env::drawGraph))
            {
                (*fct_draw)();
            }
        }
    }
    return nbAddedNode;
}*/
