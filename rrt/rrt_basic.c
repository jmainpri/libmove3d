#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
//#include "Collision-pkg.h"
#include "Rrt-pkg.h"



/*************************************************/
/* extension simple et test d'arret des 2 arbres */
/*************************************************/

int start_rrt(int pas, int Knum, 
	      int (*fct_user)(void), 
	      int dist_choice, int connect_choice, int ext_choice, 
	      int (*fct_user2)(void))
{
  configPt q, qnew;
  rrt_node **lastnode1, **lastnode2; /* last node enter in the tree */
  rrt_node *rnode1, *rnode2;	/* 2 nodes relied */
  rrt_node *tree_near;
  int j;
  int newnode;             /* booleen testant la creation d'un nouveau noeud */
  int chemin = FALSE;      /* booleen testant le chemin possible ou pas */
  p3d_rob *robotPt = (p3d_rob *)NULL;	/* current robot */
  int njnt = p3d_get_robot_njnt(); /* number of dof */
  rrt_node *temp;
  /* compteur d'appel à la méthode locale */
  int nb_lm1 = 0;		/* durant la phase d'extension de */
				/* l'abre init */
  int nb_lm2 = 0;		/* durant la phase d'extension de */
				/* l'abre goal */
  int nb_lm_connect = 0;	/* durant la phase connect2tree */

  double ts,tu;
  double env_dist;
  rrt_graph *graph; 
  /* arret de l'extention de l'un des 2 arbres */
  int bloc_tree1 = TRUE, bloc_tree2 = TRUE;
  double *tab;
  int n1 = 0, n2 = 0;		/* compteur de cette extention */

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
  tab = MY_ALLOC(double, njnt+1);
  if (dist_choice == 5)
    dist_jnt_axe(tab);


  /* allocation des variables */  

  q = p3d_alloc_config(robotPt);
  qnew = p3d_alloc_config(robotPt);

  lastnode1 = MY_ALLOC(pt_rrtnode, 1);
  lastnode2 = MY_ALLOC(pt_rrtnode, 1);
  
  temp = graph->init;
  while (temp->next != NULL)
    temp = temp->next;
  *lastnode1 = temp;   /* dernier noeud creer in graph->init */

  temp = graph->goal;
  while (temp->next != NULL)
    temp = temp->next;
  *lastnode2 = temp;   /* dernier noeud creer graph->goal */

  /* iteration de l'agrandissement de l'arbre Knum fois au maximun*/
  for (j=0; j<Knum; j++)
    {
      if (Knum == 2147483640)	/* TODO explain that ! */
	  j --;

      /* generate a random config */
      p3d_shoot(robotPt,q);

      /* call the function extend the init tree */
      if (bloc_tree1){
	if (n1 <= (2*n2))
	  {
	    tree_near = nearest_neighbor(q, graph->init, dist_choice,
					 (double *)NULL);

	    
	    newnode = FALSE;
	    newnode = extend(graph->init, q, tree_near->q, 
			     pas, &nb_lm1, ext_choice, qnew);
	    if (newnode)
	      {
		graph->nb_node ++;
		add_rrt_node(tree_near, qnew, lastnode1, graph->nb_node);
		graph->nb_node1++;
		n1 ++;
		if (fct_user2)
		  (*fct_user2)();
		if (chemin == FALSE)
		  {
		    chemin = connect2tree(lastnode1, graph->goal,
					  &rnode1, &rnode2, 
					  connect_choice,
					  &nb_lm_connect, dist_choice,
					  env_dist);
		    //		    PrintInfo(("chemin (tree1) = %d\n", chemin));
		  }
	      }
	  }
      }

      /* call the function extend the goal tree */
      if (bloc_tree2)
	if (n2 <= (2*n1))
	  { 
	    tree_near = nearest_neighbor(q, graph->goal, dist_choice,
					 (double *)NULL);
	    newnode = FALSE;     
	    newnode = extend(graph->goal, q, tree_near->q, 
			     pas, &nb_lm2, ext_choice, qnew); 
	    if (newnode) 
	      {  
		graph->nb_node ++;
		add_rrt_node(tree_near, qnew, lastnode2, graph->nb_node);
		graph->nb_node2++;
		n2 ++; 
		if (fct_user2) 
		  (*fct_user2)();
		if (chemin == FALSE)
		  { 
		    chemin = connect2tree (lastnode2, graph->init,
					   &rnode1, &rnode2,
					   connect_choice, 
					   &nb_lm_connect,
					   dist_choice, env_dist);
		    //		    PrintInfo(("chemin (tree2) = %d\n", chemin));
		  }
	      }
	  } 
      
      /* arret si chemin trouve */     
      if (chemin)
	{
	  j = Knum;
	  graph->chemin = TRUE;
	  graph->n1 = rnode1->number;
	  graph->n2 = rnode2->number;
	}

      /* arret si STOP demande */
      if(fct_user)
	if(!(*fct_user)()) 
	  j=Knum;

      /* arret eventuel de l'arbre init */
      if (n1 > 600)
	{
	  if (bloc_tree2 && bloc_tree1)
	    {
	      if (n2 < ((n1/2)+5))
		{
		  bloc_tree1 = FALSE;
		  n1 = Knum/2;      /* eviter d'avoir n1 negatif qd on le multiplie par 2 */
		}
	    }
	}
      /* arret eventuel de l'arbre goal */
      if (n2 > 600)
	{
	  if (bloc_tree2 && bloc_tree1)
	    {
	      if (n1 < ((n2/2)+5))
		{
		  bloc_tree2 = FALSE;
		  n2 = Knum/2;      /* eviter d'avoir n2 negatif */
				    /* qd on le multiplie par 2 */
		}
	    }
	}
    } /* for... */
  
  ChronoTimes (&tu, &ts);
  graph->time = graph->time + tu;
  /* nb appel methode local */
  graph->nb_lm = graph->nb_lm + nb_lm_connect + nb_lm1 + nb_lm2;
  
  if (chemin)
    {
      PrintInfo(("rrt:find path trajectory\n"));
      print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
      generate_curve(rnode1, rnode2);
    }
  else
    {
      PrintInfo(("rrt:no path trajectory\n"));
      print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
    }
  
  ChronoOff();
  p3d_destroy_config(robotPt, q);
  p3d_destroy_config(robotPt, qnew);
  MY_FREE(tab,double, njnt+1);
  MY_FREE(lastnode1, pt_rrtnode, 1);
  MY_FREE(lastnode2, pt_rrtnode, 1);
  return (chemin);
}
