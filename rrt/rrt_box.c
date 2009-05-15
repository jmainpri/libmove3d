#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
//#include "Collision-pkg.h"
#include "Rrt-pkg.h"

/* fonction locale */
static rrt_node *spec_rrt_box(rrt_graph *, rrt_node *,double *,
		    int);


/* varaible gloable */
static double env_dist;

/********************************************************************************/
/* extension simple si dist(qrand, qnear)<env_dist et test d'arret des 2 arbres */
/********************************************************************************/

int start_box(int pas, int Knum, 
	      int (*fct_user)(void), 
	      int dist_choice, int connect_choice, int ext_choice, 
	      int (*fct_user2)(void))
{
  double *q;                                          /* configuration aleatoire */
  double *qnew;
  rrt_node **lastnode1, **lastnode2;                  /* last node enter in the tree */
  rrt_node *rnode1, *rnode2;                          /* 2 nodes relied */
  rrt_node *tree_near;
  int j;
  int newnode;             /* booleen testant la creation d'un nouveau noeud */
  int chemin = FALSE;      /* booleen testant le chemin possible ou pas */
  p3d_rob *r;              /* current robot */
  int nddl = p3d_get_robot_njnt();                         /* number of dof */
  rrt_node *temp;
  int nb_lm1 = 0, nb_lm2 = 0, nb_lm_connect = 0;  /* nombre d'appel a la methode local */
  double ts,tu;
  rrt_graph *graph;
  int bloc_tree1 = TRUE, bloc_tree2 = TRUE;       /* arret de l'extention de l'un des 2 arbres */
  double *tab;
  int n1 = 0, n2 = 0;                   /* compteur de cette extention */

  graph = get_rrt_graph();
  ChronoOn();
  if (graph == NULL)
    graph = new_tree();

  set_rrt_graph(graph);
  r = graph->r;

  /* calcul de la distance maximale (robot box) de l'espace */
  /* cette distance est divise par le pas/100 */
  /* le nombre obtenu servira lors de la connection des arbres */
  env_dist =  calcul_env_dist(dist_choice, pas);
  tab = MY_ALLOC(double, nddl+1);
  if (dist_choice == 5)
    dist_jnt_axe(tab);

  /* allocation des variables */  

  q = MY_ALLOC(double, nddl+4);
  qnew = MY_ALLOC(double, nddl+4);
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
  
  /* iteration de l'agrandissement de l'arbre Knum fois au maximun*/
  for (j=0; j<Knum; j++)
    {
      if (Knum == 2147483640)
	j --;
      
      if (bloc_tree1)
	{
	  /* call the function extend the tree */
	  if (n1 <= (2*n2))
	    {
	      /* ici tree_near est choisi de manière particulière */
	      tree_near = spec_rrt_box(graph, graph->init, q, dist_choice);
	      newnode = FALSE;
	      newnode = extend (graph->init, q, tree_near->q, 
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
		    chemin = connect2tree(lastnode1, graph->goal, 
					  &rnode1, &rnode2, 
					  connect_choice,
					  &nb_lm_connect, 
					  dist_choice, env_dist);
		}
	    }
	}
      if (bloc_tree2)
	{
	  /* call the function extend the tree */
	  if (n2 <= (2*n1))
	    { 
	      /* ici tree_near est choisi de manière particulière */
	      tree_near = spec_rrt_box(graph, graph->goal, q, dist_choice);
	      newnode = FALSE;     
	      newnode = extend (graph->goal, q, tree_near->q, pas, &nb_lm2, ext_choice, qnew);
	      if (newnode) 
		{  
		  graph->nb_node ++;
		  add_rrt_node(tree_near, qnew, lastnode2, graph->nb_node);
		  graph->nb_node2++; 
		  n2 ++;
		  if (fct_user2) 
		    (*fct_user2)();
		  if (chemin == FALSE) 
		    chemin = connect2tree (lastnode2, graph->init, 
					   &rnode1, &rnode2, 
					   connect_choice,
					   &nb_lm_connect, 
					   dist_choice, env_dist);  
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
	{ 
	  if(!(*fct_user)()) j=Knum;
	}
      /* arret eventuel de l'arbre init */
      if (n1 > 600)
	{
	  if (bloc_tree2 && bloc_tree1)
	    {
	      if (n2 < ((n1/2)+5))
		{
		  bloc_tree1 = FALSE;
		  PrintInfo(("arret de tree init\n"));
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
  
  if (chemin)
    {
      generate_curve(rnode1, rnode2);
      PrintInfo(("rrt:find path trajectory\n"));
      print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
    }
  else
    {
      PrintInfo(("rrt:no path trajectory\n"));
      print_info (graph, nb_lm1, nb_lm2, nb_lm_connect);
    }
  
  ChronoOff();
  MY_FREE(q, double, nddl+4);
  MY_FREE(qnew, double, nddl+4);
  MY_FREE(tab,double, nddl+1);
  MY_FREE(lastnode1, pt_rrtnode, 1);
  MY_FREE(lastnode2, pt_rrtnode, 1);
  return (chemin);
}

/* fonction specifique à rrt_box */
static rrt_node *spec_rrt_box(rrt_graph *graph, rrt_node *start_node, 
			      double *q,
			      int dist_choice)
{
  int box = FALSE;		/* booleen testant la possibilite */
				/* d'extention (qrand in box) */
  rrt_node *tree_near;		/* la config retournée */
  double dist_qrand_qnear;

  while (box == FALSE)
    {
      /* generate a random config */
      p3d_shoot(graph->r,q);
      tree_near = nearest_neighbor(q, start_node, dist_choice, &dist_qrand_qnear);
      /*       if (dist_choice == 4) */
      /* 	dist_qrand_qnear = distance_rrt(q,tree_near->q,1); */
      /*       else */
      /* 	dist_qrand_qnear = distance_rrt(q,tree_near->q,dist_choice); */
      if (dist_qrand_qnear < env_dist)
	box = TRUE;
    }

  return tree_near;
}
