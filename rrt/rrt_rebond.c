#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Rrt-pkg.h"


/********************************************************************************/
/* pour les environnements tres contraints, on realise 2 fois 4 rebonds         */
/* pour chaque nouveau noeud et si on avance on recommence les 2 fois 4 rebonds */ 
/********************************************************************************/

int start_rrt_rebond(int pas, int Knum, int (*fct_user)(void),
		     int dist_choice, int connect_choice, 
		     int ext_choice, int (*fct_user2)(void))
{
  double *qrand;                                /* configuration aleatoire */
  double *q, *q1, *q2; 
  rrt_node **lastnode1, **lastnode2;        /* last node enter in the tree */
  rrt_node *rnode1, *rnode2;                /* 2 nodes relied */
  rrt_node *tree_near;
  int j, i, k, l;
  int chemin = FALSE;      /* booleen testant le chemin possible ou pas */
  int valide = FALSE;
  int valide2;
  p3d_rob *robotPt;              /* current robot */
  int njnt = p3d_get_robot_njnt();                         /* number of dof */
  rrt_node *temp;
  /* nombre d'appel a la methode local */
  int nb_lm1 = 0, nb_lm2 = 0, nb_lm_connect = 0;  
  double ts,tu;
  double env_dist;
  rrt_graph *graph;
  /* arret de l'extention de l'un des 2 arbres */
  int bloc_tree1 = TRUE, bloc_tree2 = TRUE;       
  double *tab;
  int n1 = 0, n2 = 0;                   /* compteur de cette extention */

  tab = MY_ALLOC(double, njnt+1);

  graph = get_rrt_graph();
  ChronoOn();
  if (graph == NULL)
    graph = new_tree();

  set_rrt_graph(graph);
  robotPt = graph->r;

  /* calcul de la distance maximale (robot box) de l'espace */
  /* cette distance est divise par le pas/100 */
  /* le nombre obtenu servira lors de la connection des arbres */
  env_dist =  calcul_env_dist(dist_choice, pas);
  if (dist_choice == 5)
    dist_jnt_axe(tab);


  //PrintInfo(("debut start rrt rebond\n"));

/* allocation des variables */  

  qrand = p3d_alloc_config(robotPt);
  q = p3d_alloc_config(robotPt);
  q1 = p3d_alloc_config(robotPt);
  q2 = p3d_alloc_config(robotPt);

  lastnode1 = MY_ALLOC(pt_rrtnode, 1);
  lastnode2 = MY_ALLOC(pt_rrtnode, 1);
  
  temp = graph->init;
  while (temp->next != NULL)
    temp = temp->next;
  *lastnode1 = temp;   /* dernier noeud creer */

  temp = graph->goal;
  while (temp->next != NULL)
    temp = temp->next;
  *lastnode2 = temp;   /* dernier noeud creer */
  //PrintInfo(("fin lastnode\n"));

  /* iteration de l'agrandissement de l'arbre Knum fois au maximun*/
  for (j=0; j<Knum; j++) {
    if (Knum == 2147483640)
      j --;
    
    if (bloc_tree1){
      if (n1 <= (2*n2)) {
	//PrintInfo(("init tree\n"));
	/* generate a random config */
	p3d_shoot(robotPt,qrand);
	tree_near = nearest_neighbor(qrand, graph->init,
				     dist_choice, (double *)NULL);
	valide = extend (graph->init, qrand, tree_near->q, 
			 pas, &nb_lm1, ext_choice, q);
	if (valide) {
	  graph->nb_node ++;
	  graph->nb_node1++;
	  n1 ++;
	  add_rrt_node(tree_near, q, lastnode1, graph->nb_node);
	  //PrintInfo(("extension init tree\n"));
	  if (fct_user2)
	    (*fct_user2)();
	  if (chemin == FALSE)
	    chemin = connect2tree(lastnode1, graph->goal, &rnode1,
				  &rnode2, connect_choice,
				  &nb_lm_connect, dist_choice,
				  env_dist);
	  
	  valide2 = TRUE;
	  while (valide2) {
	    valide2 = FALSE;
	    for (i=0; i<4; i++) {
	      p3d_shoot(robotPt,qrand);
	      valide = extend (graph->init, qrand, q, 
			       pas, &nb_lm1, ext_choice, q1);
	      if (valide) {
		//PrintInfo(("rebond(1) init tree %d\n", i));
		if (fct_user2)
		  (*fct_user2)();
		if (chemin == FALSE)
		  chemin = connect2tree(lastnode1, graph->goal, 
					&rnode1, &rnode2,
					connect_choice, 
					&nb_lm_connect, dist_choice, env_dist);
		else			  
		  i=4;
		for (k=0;k<4;k++) {
		  p3d_shoot(robotPt,qrand);
		  valide = extend (graph->init, qrand, q1, pas, &nb_lm1, ext_choice, q2);
		  if (valide) {
		    if (fct_user2)
		      (*fct_user2)();
		    //PrintInfo(("rebond(2) init tree %d\n", k));
		    if (collide_rrt(q,q2) == FALSE) {
		      graph->nb_node ++;
		      graph->nb_node1++;
		      n1 ++;
		      add_rrt_node((*lastnode1), q1, lastnode1, 
				   graph->nb_node);
		      graph->nb_node ++;
		      graph->nb_node1++;
		      n1 ++;
		      add_rrt_node((*lastnode1), q2, lastnode1, 
				   graph->nb_node);
		      if (chemin == FALSE)
			chemin =
			  connect2tree(lastnode1,
				       graph->goal,
				       &rnode1,
				       &rnode2, 
				       connect_choice, 
				       &nb_lm_connect, dist_choice, env_dist);
		      //PrintInfo(("on recommence\n"));
		      if (chemin == FALSE)
			valide2 = TRUE;
		      i = 4;
		      k = 4;
		      for (l=0; l<NDOF_BASE; l++)
			q[l] = q2[l];
		    }
		  }
		}
	      }
	    }
	  }
	}
      }
    } 
    if (bloc_tree2) {
      if (n2 <= (2*n1)) {
	//PrintInfo(("goal tree\n"));
	/* generate a random config */
	p3d_shoot(robotPt,qrand);
	tree_near = nearest_neighbor(qrand, graph->goal,
				     dist_choice, (double *)NULL);
	valide = extend (graph->goal, qrand, tree_near->q, pas, 
			 &nb_lm1, ext_choice, q);
	if (valide) {
	  graph->nb_node ++;
	  graph->nb_node2++;
	  n2 ++;
	  add_rrt_node(tree_near, q, lastnode2, graph->nb_node);
	  //PrintInfo(("extension goal tree(rebond)\n"));
	  if (fct_user2)
	    (*fct_user2)();
	  if (chemin == FALSE) 
	    chemin = connect2tree (lastnode2, graph->init, &rnode1, 
				   &rnode2, connect_choice, &nb_lm_connect, 
				   dist_choice, env_dist);  
	  valide2 = TRUE;
	  while (valide2) {
	    valide2 = FALSE;
	    for (i=0;i<4;i++) {
	      p3d_shoot(robotPt,qrand);
	      valide = extend (graph->goal, qrand, q, pas, 
			       &nb_lm2, ext_choice, q1);
	      if (valide) {
		//PrintInfo(("rebond(1) goal tree %d\n", i));
		if (fct_user2)
		  (*fct_user2)();
		if (chemin == FALSE) 
		  chemin = connect2tree (lastnode2,
					 graph->init, 
					 &rnode1, &rnode2,
					 connect_choice, 
					 &nb_lm_connect, 
					 dist_choice, 
					 env_dist);  
		else
		  i=4;
		for (k=0;k<4;k++) {
		  p3d_shoot(robotPt,qrand);
		  valide = extend (graph->goal, qrand, q1, pas, 
				   &nb_lm2, ext_choice, q2);
		  if (valide) {
		    //PrintInfo(("rebond(2) goal tree %d\n",k));
		    if (fct_user2)
		      (*fct_user2)();
		    if (collide_rrt(q,q2) == FALSE) {
		      graph->nb_node ++;
		      graph->nb_node2++;
		      n2 ++;
		      add_rrt_node((*lastnode2), q1, lastnode2, 
				   graph->nb_node);
		      graph->nb_node ++;
		      graph->nb_node2++;
		      n2 ++;
		      add_rrt_node((*lastnode2), q2, lastnode2, 
				   graph->nb_node);
		      if (chemin == FALSE) 
			chemin = connect2tree(lastnode2, graph->init, 
					      &rnode1, &rnode2,
					      connect_choice, 
					      &nb_lm_connect, 
					      dist_choice, env_dist);  
		      //PrintInfo(("on recommence\n"));
		      if (chemin == FALSE)
			valide2 = TRUE;
		      i = 4;
		      k = 4;
		      for (l=0; l<NDOF_BASE; l++)
			q[l] = q2[l];
		    }
		  }
		}
	      }
	    }
	  }
	}
      }
    }
    /* arret si chemin trouve */     
    if (chemin) {
      j = Knum;
      graph->chemin = TRUE;
      graph->n1 = rnode1->number;
      graph->n2 = rnode2->number;
    }
    /* arret si STOP demande */
    if(fct_user) {
      if(!(*fct_user)()) j=Knum;
    }
    /* arret eventuel de l'arbre init */
    if (n1 > 600) {
      if (bloc_tree2 && bloc_tree1) {
	if (n2 < ((n1/2)+5)) {
	  bloc_tree1 = FALSE;
	  PrintInfo(("arret de tree init\n"));
	  n1 = Knum/2;      /* eviter d'avoir n1 negatif qd on le multiplie par 2 */
	}
      }
    }
    /* arret eventuel de l'arbre goal */
    if (n2 > 600) {
      if (bloc_tree2 && bloc_tree1) {
	if (n1 < ((n2/2)+5)) {
	  bloc_tree2 = FALSE;
	  PrintInfo(("arret de tree goal\n"));
	  n2 = Knum/2;      /* eviter d'avoir n2 negatif qd on le multiplie par 2 */
	}
      }
    }
  }
  ChronoTimes (&tu, &ts);
  graph->time = graph->time + tu;
  /* nombre d'appel a la methode local */
  graph->nb_lm = graph->nb_lm + nb_lm_connect + nb_lm1 + nb_lm2;  
  
  if (chemin) {
    generate_curve(rnode1, rnode2);
    PrintInfo(("rrt:find path trajectory\n"));
    print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
  }
  else {
    PrintInfo(("rrt:no path trajectory\n"));
    print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
  }
  
  ChronoOff();
  p3d_destroy_config(robotPt, qrand);
  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, q1);
  p3d_destroy_config(robotPt, q2);
  MY_FREE(tab, double, njnt+1);
  MY_FREE(lastnode1,pt_rrtnode, 1);
  MY_FREE(lastnode2,pt_rrtnode, 1);
  return(chemin);
}
