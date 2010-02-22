/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "planners_cxx.hpp"
#include "plannerFunctions.hpp"
#include "API/Trajectory/CostOptimization.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_CostSpace/HRICS_Planner.h"
#include "HRI_CostSpace/RRT/HRICS_rrt.h"
#include "HRI_CostSpace/HRICS_CSpace.h"
#include "HRI_CostSpace/RRT/HRICS_rrtPlan.h"
#endif

using namespace std;

int p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{
    ENV.setBool(Env::isRunning,true);

    double tu,ts;

    GraphPt = GraphPt ? GraphPt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
    RRT* rrt = (RRT*)plannerlist[0];
#else
    Robot* _Robot = new Robot(GraphPt->rob);
    Graph* _Graph = new Graph(_Robot,GraphPt);

    RRT* rrt;

    // Initialize all RRTs
    if(ENV.getBool(Env::isManhattan))
    {
        rrt = new ManhattanLikeRRT(_Robot,_Graph);
    }
#ifdef HRI_COSTSPACE
    else if(ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::HRIPlannerTRRT))
    {
        rrt = new HRICS_RRT(_Robot,_Graph);
    }
    else if(ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::HRIPlannerTRRT))
    {
        rrt = new HRICS_RRTPlan(_Robot,_Graph);
    }
#endif
    else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::useTRRT) )
    {
        rrt = new TransitionRRT(_Robot,_Graph);
    }
    else
    {
        if( ENV.getBool(Env::isCostSpace) && (!ENV.getBool(Env::useTRRT)) )
        {
            ENV.setBool(Env::isCostSpace,false);
        }

        rrt = new RRT(_Robot,_Graph);
    }
#endif

    int nb_added_nodes = rrt->init();

    _Graph = rrt->getActivGraph();

    // Main Run functions of all RRTs
    nb_added_nodes += rrt->run();

    ChronoTimes(&(_Graph->getGraphStruct()->rrtTime), &ts);
    printf("graph time = %f\n",_Graph->getGraphStruct()->rrtTime);
    printf("nb added nodes %d\n", nb_added_nodes);
    printf("nb nodes (Wrapper) %d\n",_Graph->getNodes().size());
    printf("nb nodes %d\n",_Graph->getGraphStruct()->nnode);

    bool res = rrt->trajFound();

    if( (!ENV.getBool(Env::isCostSpace)) && (!ENV.getBool(Env::useTRRT)) )
    {
        ENV.setBool(Env::isCostSpace,true);
    }

    // Smoothing phaze
    if(res)
    {
        ENV.setBool(Env::isRunning,true);
        p3d_ExtractBestTraj(_Graph->getGraphStruct());

        if(ENV.getBool(Env::withSmoothing))
        {
            CostOptimization optimTrj(_Robot,_Robot->getTrajStruct());

            _Graph->getGraphStruct()->rrtCost1 = optimTrj.cost();

            if(ENV.getBool(Env::withDeformation))
            {
                ENV.setBool(Env::FKShoot,true);
                optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize));
                ENV.setBool(Env::FKShoot,false);
            }

            _Graph->getGraphStruct()->rrtCost2 = optimTrj.cost();

            if(ENV.getBool(Env::withShortCut))
            {
                optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
            }
            optimTrj.replaceP3dTraj();
        }
    }

    ENV.setBool(Env::isRunning,false);

#ifndef LIST_OF_PLANNERS
    delete rrt;
#endif
    if(res)
        return _Graph->getGraphStruct()->nnode;
    else
        return false;
}


bool p3d_run_est(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();


	printf("------------- Running EST --------------\n");

#ifdef LIST_OF_PLANNERS
	RRT* rrt = (RRT*)plannerlist[0];
#else
        Robot* _Robot = new Robot(GraphPt->rob);
        Graph* _Graph = new Graph(_Robot,GraphPt);

	EST* est;

        est = new EST(_Robot,_Graph);
#endif

	int nb_added_nodes = est->init();

	_Graph = est->getActivGraph();

	printf("nb nodes %zu\n",_Graph->getNodes().size());

	nb_added_nodes += est->run();
        ENV.setBool(Env::isRunning,false);

	printf("nb added nodes %d\n", nb_added_nodes);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	bool res = est->trajFound();

#ifndef LIST_OF_PLANNERS
	delete est;
#endif

	return res;
}

int p3d_run_prm(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
        int ADDED;

        GraphPt = GraphPt ? GraphPt : p3d_create_graph();
        cout << "Create Robot and Graph " << endl;

#ifdef LIST_OF_PLANNERS
        PRM* prm = (PRM*)plannerlist[2];
#else
        Robot* _Robot = new Robot(GraphPt->rob);
        Graph* _Graph = new Graph(_Robot,GraphPt);

        PRM* prm = new PRM(_Robot,_Graph);
#endif
        cout << "Initializing PRM " << endl;
        ADDED = prm->init();

        cout << "Expanding PRM " << endl;
        ADDED += prm->expand();

        printf("nb added nodes %d\n", ADDED);
        printf("nb nodes %zu\n",_Graph->getNodes().size());
        *fail = !prm->trajFound();

#ifndef LIST_OF_PLANNERS
        delete prm;
#endif

        return ADDED;
}


int p3d_run_vis_prm(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
	int ADDED;

        GraphPt = GraphPt ? GraphPt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
	Vis_PRM* vprm = (Vis_PRM*)plannerlist[1];
#else
        Robot* _Robot = new Robot(GraphPt->rob);
        Graph* _Graph = new Graph(_Robot,GraphPt);

        Vis_PRM* vprm = new Vis_PRM(_Robot,_Graph);
#endif

	ADDED = vprm->init();

        ADDED += vprm->expand(GraphPt, fct_stop, fct_draw);

	printf("nb added nodes %d\n", ADDED);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	*fail = !vprm->trajFound();

#ifndef LIST_OF_PLANNERS
	delete vprm;
#endif

	return ADDED;
}

int p3d_run_acr(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
	int ADDED;

        GraphPt = GraphPt ? GraphPt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
	ACR* acr = (ACR*)plannerlist[3];
#else
        Robot* _Robot = new Robot(GraphPt->rob);
        Graph* _Graph = new Graph(_Robot,GraphPt);

        ACR* acr = new ACR(_Robot,_Graph);
#endif

	ADDED = acr->init();

        ADDED += acr->expand(GraphPt, fct_stop, fct_draw);

	printf("nb added nodes %d\n", ADDED);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	*fail = !acr->trajFound();

#ifndef GLOBAL
	delete acr;
#endif

	return ADDED;
}


/**
 * Function To replace p3d_Learn in Case of C++ API Use
 */
void p3d_learn_cxx(int NMAX,
		int (*fct_stop)(void), void (*fct_draw)(void)) {
	p3d_graph *G;
	int inode, ADDED;
	double tu, ts;
	int fail = 1;
	int nbInitGraphNodes, nbGraphNodes;

	ChronoOn();

	if (!XYZ_GRAPH) G = p3d_create_graph();
	else           G = XYZ_GRAPH;
	/*debut modif fpilarde*/
	inode = 0;
	p3d_set_planning_type(P3D_GLOBAL);
	ENV.setBool(Env::expandToGoal,false);

	if (p3d_get_MOTION_PLANNER() != P3D_DIFFUSION) {
		//while (inode < NMAX) {
		/* Call basic PRM or Visibility method */
		switch (p3d_get_MOTION_PLANNER()) {

		case 1:
			cout << "CXX_PLANNER c++ API : p3d_run_prm" << endl;
			ADDED = p3d_run_prm(G, &fail, fct_stop, fct_draw);
			break;

		case 2:
			cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
			ADDED = p3d_run_vis_prm(G, &fail, fct_stop, fct_draw);
			break;

		case P3D_ALL_PRM:
			cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
			ADDED = p3d_run_acr(G, &fail, fct_stop, fct_draw);
			break;

		default:
			PrintInfo(("p3d_learn : ERREUR : pas de planificateur global...\n"));
			return;
		}
		inode += ADDED;
	}

	else {
		nbInitGraphNodes = G->nnode;
		ADDED = p3d_run_rrt(G, fct_stop, fct_draw);
		nbGraphNodes = G->nnode;
		inode  = nbGraphNodes - nbInitGraphNodes;
	}

	p3d_set_planning_type(P3D_NONE);
	PrintInfo(("Pour la creation de %d noeuds : ", inode));

	ChronoTimes(&tu, &ts);
	G->time = G->time + tu;
	/* When retrieving statistics;
  	        			Commit Jim; date: 01/10/2008 */
	if(getStatStatus()){
		G->stat->preTime += tu;
	}
	ChronoPrint("");
	ChronoOff();

	MY_ALLOC_INFO("After p3d_learn");
	p3d_print_info_graph(G);
}

/**
 * Function To replace p3d_specific_learn in Case of C++ API Use
 */
int p3d_specific_learn_cxx(double *qs, double *qg, int *iksols, int *iksolg,
		int (*fct_stop)(void), void (*fct_draw)(void)) {

	p3d_graph *G;

	int       inode = 0, fail = 1, ADDED;
	double    tu, ts;

	int nbInitGraphNodes, nbGraphNodes;
#ifdef ENERGY
	int n_coldeg, icoldeg;
	double *coldeg_qs;
#endif
	/* Avoid some stupid errors */
	if (qs == NULL) {
		PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration initiale\n"));
		return(FALSE);
	}

	if ((qg == NULL) && (ENV.getBool(Env::expandToGoal) == true)) {
		PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration finale\n"));
		return(FALSE);
	}

	ENV.setBool(Env::expandToGoal,true);

	ChronoOn();

	if (!XYZ_GRAPH)  G = p3d_create_graph();
	else            G = XYZ_GRAPH;

	if (p3d_GetIsWeightedChoice() == TRUE) {
		p3d_init_root_weight(G);
	}
	if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH) {
		p3d_init_pb_variables(G);

#ifdef ENERGY
		if (p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) {
			n_coldeg = bio_get_num_collective_degrees();
			// init coldeg_q in Ns
			coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
			for (icoldeg = 0; icoldeg < n_coldeg; icoldeg++) {
				coldeg_qs[icoldeg] = 0.0;
			}
			bio_copy_coldeg_q_in_N(Ns, coldeg_qs, n_coldeg);
			bio_destroy_coldeg_config(coldeg_qs, n_coldeg);
			// WARNING : currently Ng is not considered !!!
		}
#endif


	}
	ADDED = TRUE;
	if (p3d_get_MOTION_PLANNER() != P3D_DIFFUSION) {
		/* While solution does not exists, insert new nodes with basic PRM or Visibility or RRT */
		//     while ((Ns->numcomp != Ng->numcomp) && !p3d_compco_linked_to_compco(Ns->comp, Ng->comp)) {  modif fpilarde
		switch (p3d_get_MOTION_PLANNER()) {

		case P3D_BASIC:
			cout << "CXX_PLANNER c++ API : p3d_run_prm" << endl;
			ADDED = p3d_run_prm(G, &fail, fct_stop, fct_draw);
			break;

		case P3D_ISOLATE_LINKING:
			cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
			ADDED = p3d_run_vis_prm(G, &fail, fct_stop, fct_draw);
			break;

		case P3D_ALL_PRM:
			cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
			ADDED = p3d_run_acr(G, &fail, fct_stop, fct_draw);
			break;

#ifdef ENERGY
		case BIO_COLDEG_RRT:
			ADDED = bio_expand_coldeg_rrt(G, fct_stop);
			break;
#endif
		default:
			PrintInfo(("p3d_specific_learn : ERREUR : pas de planificateur global...\n"));
			return(FALSE);
		}

		inode += ADDED;

	} else {
		nbInitGraphNodes = G->nnode;
		cout << "CXX_PLANNER c++ API : p3d_run_rrt" << endl;
		ADDED = p3d_run_rrt(G, fct_stop, fct_draw);
		nbGraphNodes = G->nnode;
		inode  = nbGraphNodes - nbInitGraphNodes;
	}
	p3d_set_planning_type(P3D_NONE);
	PrintInfo(("Pour la creation de %d noeuds : ", inode));
	ChronoPrint("");

	ChronoTimes(&tu, &ts);
	G->time = G->time + tu;
	/* When retrieving statistics;
  	        			Commit Jim; date: 01/10/2008 */
	if(getStatStatus()){
		G->stat->preTime += tu;
	}
	ChronoOff();

	p3d_print_info_graph(G);
	MY_ALLOC_INFO("After p3d_specific_learn");
	if (p3d_get_saveInfoInFile()) {
		//    save_infos_in_file(G, ADDED);
	}

	PrintInfo(("\n"));
	return(ADDED || !fail);
}
