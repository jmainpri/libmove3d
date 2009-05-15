#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Rrt-pkg.h"


#define sqr(x) ((x)*(x))



/**************************/
/* variable globale arbre */
/**************************/

static rrt_graph *G_RRT = NULL;


/**************************************/
/* definition des fonctions utilisees */
/**************************************/


void free_alloc_rrt(rrt_node *tree);

static void free_pile (q_pile *pile);

rrt_graph *get_rrt_graph(void);
void reset (void);


/*****************************/
/* choice of start extention */
/*****************************/

int start(int pas, int Knum, int (*fct_user)(void), 
	  int dist_choice, int connect_choice, int ext_choice, 
	  int (*fct_user2)(void), int start_choice)
{
  int chemin = 0;

/*   PrintInfo(("pas = %d, dist_choice = %d , connect_choice = %d, ext_choice = %d,  
 	  start_choice = %d", pas, dist_choice, connect_choice, ext_choice, start_choice)); */

  /* pour ne pas chercher inutilement... */
  if(p3d_equal_config((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT),
		      ((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT))->ROBOT_POS,
		      ((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT))->ROBOT_GOTO))
    {
      PrintInfo(("ROBOT_POS = ROBOT_GOTO !!!\n"));
      return 0;			/* pas de chemin ! */
    }

  switch (start_choice)
    {
    case 1:
      chemin = start_rrt(pas, Knum, (*fct_user), 
			 dist_choice, connect_choice, 
			 ext_choice, (*fct_user2));
      break;
    case 2:
      chemin = start_rrt_rebond(pas, Knum, (*fct_user), 
				dist_choice, connect_choice, 
				ext_choice, (*fct_user2));
      break;
    case 3:
      chemin = start_box(pas, Knum, (*fct_user), 
			 dist_choice, connect_choice, 
			 ext_choice, (*fct_user2));
      break;

    default:
      PrintInfo(("erreur dans le choix de la fonction start\n"));
    } 
  return (chemin);
}


/*******************/
/* nouveaux arbres */
/*******************/

rrt_graph *new_tree(void)
{
  /* current robot */
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  rrt_graph *graph;

    PrintInfo(("creation de INIT_TREE et GOAL_TREE\n"));
    graph = MY_ALLOC(rrt_graph, 1);
    graph->r = robotPt;
    graph->nb_node = 2;		/* the init and the goal conf */
    graph->nb_node1 = 1;	/* the init conf */
    graph->nb_node2 = 1;	/* the goal cong */
    graph->nb_lm = 0;
    graph->time = 0;
    graph->nb_col = 0;
    graph->chemin = FALSE;
    graph->n1 = 0;
    graph->n2 = 0;

    graph->init = MY_ALLOC(rrt_node, 1);
    graph->goal = MY_ALLOC(rrt_node, 1);
    graph->init->q = p3d_alloc_config(robotPt);
    graph->goal->q = p3d_alloc_config(robotPt);

    /* initialisation des arbres */
    graph->init->fedge = NULL;
    graph->init->precedge = NULL;
    graph->init->prec = NULL;
    graph->init->next = NULL;

    graph->goal->fedge = NULL;
    graph->goal->precedge = NULL;
    graph->goal->prec = NULL;
    graph->goal->next = NULL;
  
    /* initialisation de la structure de l'arbre */
    /* recuperation des positions initial et but du robot */
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &graph->init->q);
    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &graph->goal->q);
    
    graph->init->number = 1;
    graph->goal->number = 2; 
    return(graph);
}


/*********************************/
/* affiche les infos sur l'arbre */
/*********************************/

void print_info (rrt_graph *graph, int nb_lm1, int nb_lm2, int nb_lm_connect)
{
    PrintInfo(("time cpu rrt: %f\n",graph->time));
    PrintInfo((" initial tree:\n"));
    PrintInfo(("   number of nodes               : %d\n", graph->nb_node1));
    PrintInfo(("   number of local method calls  : %d\n", nb_lm1));
    PrintInfo((" goal tree:\n"));
    PrintInfo(("   number of nodes               : %d\n", graph->nb_node2));
    PrintInfo(("   number of local method calls  : %d\n", nb_lm2));
    PrintInfo((" number of local method with the connect function: %d\n", 
	   nb_lm_connect));
    PrintInfo(("number of collision checker calls: %d\n", graph->nb_col));
    PrintInfo(("cumulate number of local method calls: %d\n", graph->nb_lm));
    PrintInfo(("\n"));
}



/****************************************************/
/* find the nearest neighbor of q in the graph tree */
/****************************************************/

/*
  dist : la distance entre q et le rrt_node retourné
*/
rrt_node *nearest_neighbor(configPt q, rrt_node *tree, int dist_choice,
			   double *dist)
{
  double min;
  double distance;
  rrt_node *nextnode = tree;
  rrt_node *tree_near = tree;

  /* initialisation */  
  min = distance_rrt(tree->q, q, dist_choice);
  
  /* find the minimun distance config from the ramdom  config */ 
  while (nextnode != (rrt_node *)NULL)
    {
      distance = distance_rrt(q, nextnode->q, dist_choice);
      if (distance < min)
	{
	  min = distance;
	  tree_near = nextnode;
	}
      nextnode = nextnode->next;
    }

  if(p3d_equal_config((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT),
		      q, tree_near->q))
    PrintInfo(("erreur %d\n", __LINE__));


  if(dist)
    *dist = min;			/* ça peut être utile... */

  return (tree_near);
}

/****************************************************/
/* the next function tests if there is a local path */
/* between 2 configs without a collision ************/
/****************************************************/
/* 
   renvoi FALSE s'il y a collision, TRUE sinon
*/

int collide_rrt (configPt q1, configPt q2)
{
  /* courbe locale genere entre les 2 config */
  pp3d_localpath c = (pp3d_localpath)NULL;
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  int collision;		/* val return */
  int ntest=0;
  rrt_graph *g_rrt = get_rrt_graph();

  /* generate the local curve between the 2 configs */
  if(r == (p3d_rob *)NULL)
    PrintInfo(("erreur %d\n", __LINE__));

  c = p3d_local_planner(r, q1, q2); /* comptabilisé ds la fct qui appel */
  
  if(c==(pp3d_localpath)NULL)
    {
      PrintError(("localpath NULL !!!\n"));
      return FALSE;		/* chemin non valide */
    }

  /* collision test in this trajectory */
  if (p3d_col_test_localpath(r, c, &ntest))
    collision = FALSE;
  else 
    collision = TRUE;

  g_rrt->nb_col = g_rrt->nb_col + ntest;

  /* liberation du localpath */
  c->destroy(r, c);		/* TODO all is destroy ??? */

  return (collision);
}

/*********************************/
/* ajout d'un noeud dans l'arbre */  
/*********************************/

void add_rrt_node (rrt_node *cur_tree, double *qnew, 
		   rrt_node **lastnode, int number)
{
  p3d_rob *robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  rrt_edge *temp_edge1, *temp_edge2;          /* temporary new edge */
  rrt_edge *create_edge;
  rrt_node *create_node;
  rrt_graph *g_rrt;

  g_rrt = get_rrt_graph();
  create_edge = MY_ALLOC(rrt_edge, 1);
  create_node = MY_ALLOC(rrt_node, 1);
  create_node->q = p3d_alloc_config(robotPt);

  /* create new node */

  //PrintInfo(("debut creer node\n"));
  create_node->number = number;
  p3d_copy_config_into(robotPt, qnew, &create_node->q);

  create_node->fedge = NULL;
  create_node->precedge = create_edge;
  create_node->prec = (*lastnode);
  create_node->next = NULL;
  (*lastnode)->next = create_node;
  (*lastnode)  = (*lastnode)->next;

  /* create new edge */
  
  //PrintInfo(("debut creer edge\n"));

  temp_edge1 = cur_tree->fedge;
  temp_edge2 = NULL;

  while (temp_edge1 != NULL){
    temp_edge2 = temp_edge1;
    temp_edge1 = temp_edge1->next;}

  create_edge->n1 = cur_tree;
  create_edge->n2 = create_node;
  if (temp_edge2 ==  NULL){
    cur_tree->fedge = create_edge;
    create_edge->prec = NULL;
    create_edge->next = NULL;}
  else{
    temp_edge2->next = create_edge;
    create_edge->prec = temp_edge2;
    create_edge->next = NULL;}

  //PrintInfo(("sortie creer node\n"));
}



/**************************************************************/
/* liberation de la memoire lorsque l'on quitte l'application */
/**************************************************************/

void free_alloc_rrt(rrt_node *tree)
{
  p3d_rob *robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  rrt_edge *edge;
  rrt_edge *temp_edge;
  rrt_node *list;
  rrt_node *temp_node;
  int f_edge = 0, f_node = 0, f_q = 0;

  list = tree;

  while (list != NULL)
    {
      if (list->fedge != NULL)
	{
	  //PrintInfo(("del edge start\n"));
	  temp_edge = list->fedge;
	  while (temp_edge != NULL)
	    {
	      //PrintInfo(("E\n"));
	      edge = temp_edge;
	      temp_edge = temp_edge->next;
	      MY_FREE(edge, rrt_edge, 1);
	      f_edge++;
	      edge = NULL;
	    }
	  //PrintInfo(("del edge end\n"));
	}
	list = list->next;
    }
 

 list = tree;
 while (list != NULL)
   {
     //PrintInfo(("del node start\n"));
     p3d_destroy_config(robotPt, list->q);
     f_q ++;
     temp_node = list;
     list = list->next;
     MY_FREE (temp_node, rrt_node, 1);
     f_node ++;
     temp_node = NULL;
     //PrintInfo(("del node end\n"));
   }
 
 tree = NULL;
 PrintInfo(("on libere %d nodes, %d edges, %d q\n", f_node, f_edge, f_q));
}

/*
=====================================
generate_curve
=====================================
*/
/*
 * generate the trajectory from the start config to the goal config
 *
 * input : les deux dernier rrt_node relié (cf rrt_basic.c, ...)
 *
 * remarque :
 * à priori on ne sait pas à quel arbre node1 et node2 appartiennent.
 */
void generate_curve(rrt_node *node1, rrt_node *node2)
{
  rrt_node *node;
  char *traj;
  int test;
  p3d_localpath *c;
  int *nmail = MY_ALLOC(int,1);
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); /* current robot */
  rrt_graph *g_rrt;
  int i = FALSE;		/* booleen pour savoir quel */
				/* est le point de depart */
  q_pile *pile1;
  q_pile *pile2;
  q_pile *newpile, *temp;
  q_pile *pile_start = NULL;
  q_pile *pile_end = NULL;

  g_rrt = get_rrt_graph();
  pile1 = MY_ALLOC (q_pile, 1);
  pile2 = MY_ALLOC (q_pile, 1);

  /* initialisation */
  pile1->next = NULL;
  pile1->q = node1->q;
  pile1->prec = NULL;
  pile2->next = NULL;
  pile2->q = node2->q;
  pile2->prec = NULL;

  traj = MY_ALLOC(char, 10);
  strcpy(traj,"rrt_traj");


  /* creation d'une trajectoire */
  p3d_beg_desc(P3D_TRAJ, traj);

  /* On remonte l'arbre auquel node1 appartient */
  /* et on met les rrt_node dans pile1 */
  temp = pile1;
  node = node1;
  while (node != g_rrt->init && node != g_rrt->goal)
    {
      newpile = MY_ALLOC (q_pile, 1);
      temp->next = newpile;
      newpile->q = node->precedge->n1->q;
      newpile->next = NULL;
      newpile->prec = temp;
      temp = temp->next;
      node = node->precedge->n1;
    }
  /* où suis je arrivé */
  if (node == g_rrt->init)
    {
      pile_start = temp;
      i=TRUE;
    }
  if (node == g_rrt->goal)
    pile_end = temp;

  /* On remonte l'arbre auquel node2 appartient */
  /* et on met les rrt_node dans pile2 */
  temp = pile2;
  node = node2;

  while (node != g_rrt->init && node != g_rrt->goal)
    {
      newpile = MY_ALLOC (q_pile, 1);
      temp->next = newpile;
      newpile->q = node->precedge->n1->q;
      newpile->next = NULL;
      newpile->prec = temp;
      temp = temp->next;
      node = node->precedge->n1;
    }
  /* où suis-je arrivé ? */
  if (node == g_rrt->init)
    {
      pile_start = temp;
      i=FALSE;
    }
  if (node == g_rrt->goal)
    pile_end = temp;

  /* on dépile pile_start... */
  temp = pile_start;
  while (temp != pile1 && temp != pile2)
    {
      /* generate the local curve between the 2 configs */
      if(temp->q == temp->prec->q)
	PrintInfo(("erreur %d\n", __LINE__));

      c = p3d_local_planner(r, temp->q, temp->prec->q);
      test = p3d_add_desc_courbe(c);
      temp = temp->prec;
    }

  /* je construis le chemin local entre */
  /* node1 et node2 dans le bon sens... */
  if (i)
    {
      if(node1->q == node2->q)
	PrintInfo(("erreur %d\n", __LINE__));
      
      c = p3d_local_planner(r, node1->q, node2->q);
    }
  else
    {
      if(node1->q == node2->q)
	PrintInfo(("erreur %d\n", __LINE__));

      c = p3d_local_planner(r, node2->q, node1->q);
    }
  test = p3d_add_desc_courbe(c);

  /* je "dépile" en partant du début... */
  /* pile_end */
  if (i)
    temp = pile2;
  else
    temp = pile1;

  while (temp != pile_end)
    {
      /* generate the local curve between the 2 configs */
      c = p3d_local_planner(r, temp->q, temp->next->q);
      test = p3d_add_desc_courbe(c);
      temp = temp->next;
    }

  /* cloture de la trajectoire */
  test = p3d_end_desc();
  MY_FREE(traj,char, 10);
  MY_FREE(nmail,int,1);
  free_pile (pile1);
  free_pile (pile2);
}

/*****************************************/
/* liberer les piles de creation de traj */
/*****************************************/

static void 
free_pile (q_pile *pile)
{
  q_pile *temp, *temp2;

  temp = pile;
  while (temp)
    {
      temp2 = temp;
      temp = temp->next;
      MY_FREE (temp2, q_pile, 1);
    }
}


/*****************************/
/* retourne le graph courant */
/*****************************/

rrt_graph *get_rrt_graph(void)
{
  return (G_RRT);
}

void set_rrt_graph(rrt_graph *graph)
{
  G_RRT = graph;
}


/***************/
/* reset graph */
/***************/

void reset (void)
{
if (G_RRT != NULL)  
  {
    free_alloc_rrt(G_RRT->init);
    free_alloc_rrt(G_RRT->goal);
    MY_FREE (G_RRT, rrt_graph, 1);
    G_RRT = NULL;
  }
}

