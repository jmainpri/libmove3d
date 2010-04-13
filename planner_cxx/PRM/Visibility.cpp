//
// C++ Implementation: vis_prm
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "Visibility.hpp"

using namespace std;
using namespace tr1;

Vis_PRM::Vis_PRM(Robot* R, Graph* G)
        : PRM(R,G)
{
	m_nbOfExpand = 0;
    cout << " New Visibility PRM "  << endl;
}

Vis_PRM::~Vis_PRM()
{
	
}

/**
 * Is Linking Orphan node
 */
bool Vis_PRM::linkOrphanLinking(Node* N, int type, unsigned int* ADDED, int* nb_fail)
{
    int link = 0;
    vector<Node**> vect = _Graph->isOrphanLinking(N, &link);
    if ((type == 1 || type == 2) && link > 1)
    {
        for (uint k = 0; k < vect.size(); k = k + 1)
        {
            (*(vect[k]))->connectNodeToCompco(N, 0);
        }
        _Graph->insertNode(N);
        *ADDED = *ADDED + 1;
        *nb_fail = 0;
        if (ENV.getBool(Env::drawGraph))
        {
            (*_draw_func)();
        }
        vect.clear();
        return false;
    }
    else if ((type == 0 || type == 2) && link == 0)
    {
        _Graph->insertNode(N);
        *ADDED = *ADDED + 1;
        *nb_fail = 0;
        if (ENV.getBool(Env::drawGraph))
        {
            (*_draw_func)();
        }
        vect.clear();
        return true;
    }
    else
    {
        N->deleteCompco();
        MY_FREE(N->getNodeStruct(),p3d_node,1);
        N->~Node();
        vect.clear();
        return false;
    }
}


/**
 * Create Orphan Linkin node
 */
int Vis_PRM::createOrphansLinking(unsigned int nb_node, int type)
{
    unsigned int ADDED = 0;
    int nb_try = 0;
	
    while ((*_stop_func)() &&  nb_try < ENV.getInt(Env::NbTry) && (_Graph->getGraphStruct()->ncomp > 1 || !type))
    {
		if (!(_Graph->getNbNode() < nb_node)) 
		{
			return ADDED;
		}
		
        createOneOrphanLinking(type, &ADDED, &nb_try);
    }
    return ADDED;
}

/**
 * Create One Orphan Linking
 */
void Vis_PRM::createOneOrphanLinking(int type, unsigned int* ADDED, int* nb_fail)
{
    shared_ptr<Configuration> C = _Robot->shoot();
    *nb_fail = *nb_fail + 1;
    if (C->setConstraints() && !C->IsInCollision())
    {
        Node* N = new Node(_Graph, C);
        linkOrphanLinking(N, type, ADDED, nb_fail);
    }
    //   else
    //   {
    //     C->Clear();
    //     C->~Configuration();
    //   }
}

void Vis_PRM::expandOneStep()
{
	createOneOrphanLinking(2,
						   &m_nbAddedNode, 
						   &_nbConscutiveFailures);
}
