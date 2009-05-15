#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
//#include "Localpath-pkg.h"
//#include "Collision-pkg.h"
#include "Rrt-pkg.h"

static int test2tree (rrt_node **node, rrt_node *tree, 
		      rrt_node **rnode1, rrt_node **rnode2, 
		      int *nb_lm, double env_dist, int dist_choice);
static int test2near (rrt_node **node, rrt_node *tree, 
		      rrt_node **rnode1, rrt_node **rnode2, 
		      int *nb_lm, int dist_choice);
static int test2near2 (rrt_node **node, rrt_node *tree, 
		       rrt_node **rnode1, rrt_node **rnode2, 
		       int *nb_lm, int dist_choice, double env_dist);


/************************************************/
/* choix de la fonction connection des 2 arbres */
/************************************************/

/*
  input :
     - **node : the node we try to connect to tree
     - *tree : the tree we try to connect to node (init or goal...)
     - connect_choice : the methode to connect two trees,
     - dist_choice : the methode to compute the distance between to
     node,
     - 
*/
int connect2tree(rrt_node **node, rrt_node *tree, 
		 rrt_node **rnode1, rrt_node **rnode2, 
		 int connect_choice, int *nb_lm, 
		 int dist_choice, double env_dist)
{
  int link=0;
  int temp_choice;		/* distance max q change, */
				/* pas parlante pour la connexion des arbres */

  temp_choice = dist_choice;
  if (dist_choice == 4)
    temp_choice = 1;
  switch (connect_choice)
    {
    case 1:
      link = test2tree(node, tree, rnode1, rnode2, 
		       nb_lm, env_dist, temp_choice);
      break;
    case 2:
      link = test2near(node, tree, rnode1, rnode2, nb_lm, temp_choice);
      break;
    case 3:
      link = test2near2(node, tree, rnode1, rnode2, 
			nb_lm, temp_choice, env_dist);
      break;
    default:
      PrintInfo(("mauvais choix de la fonction connection des 2 arbres\n"));
      break;
    }
  return (link);
}


/***************************************************/
/* test la connection possible ou pas des 2 arbres */
/***************************************************/

static int test2tree (rrt_node **node, rrt_node *tree, 
		      rrt_node **rnode1, rrt_node **rnode2, 
		      int *nb_lm, double env_dist, int dist_choice)
{
  rrt_node *temp = tree;
  int link = FALSE;   /* booleen testant le lien entre les 2 arbres */
  
  while (temp != NULL){
    /* on teste si les noeuds ne sont pas trop distants */
    if (distance_rrt((*node)->q,temp->q,dist_choice)<env_dist)
      {
	(*nb_lm)++;
	/* test de collision */
	if (collide_rrt ((*node)->q, temp->q))
	  {
	    *rnode1 = *node;
	    *rnode2 = temp;
	    link = TRUE;
	  }
      }
    temp = temp->next;
    if (link)
      temp= NULL;
  }
  
  return (link);
}


/***************************************************/
/* test la connection possible ou pas des 2 arbres */
/***************************************************/

static int test2near (rrt_node **node, rrt_node *tree, 
		      rrt_node **rnode1, rrt_node **rnode2, 
		      int *nb_lm, int dist_choice)
{
  rrt_node *temp;
  int link;   /* booleen testant le lien entre les 2 arbres */
  
  link = FALSE;
  temp = nearest_neighbor((*node)->q, tree, dist_choice, (double *)NULL);
  
  (*nb_lm)++;
  /* test de collision */
  if (collide_rrt ((*node)->q, temp->q))
    {
     *rnode1 = *node;
     *rnode2 = temp;
     link = TRUE;
    }
  
  return (link);
}


/***************************************************/
/* test la connection possible ou pas des 2 arbres */
/***************************************************/

static int test2near2 (rrt_node **node, rrt_node *tree, 
		       rrt_node **rnode1, rrt_node **rnode2, 
		       int *nb_lm, int dist_choice, double env_dist)
{
  rrt_node *temp = tree;
  int link = FALSE;      /* booleen testant le lien entre les 2 arbres */
  int proche = FALSE;    /* booleen test si il y a un noeud */
			 /* proche dans l'autre arbre */
  
  /* on teste si les noeuds ne sont pas trop distants */
  while (temp != NULL)
    {
      if (distance_rrt((*node)->q,temp->q, dist_choice) < env_dist)
	proche = TRUE;
      temp = temp->next;
      if (proche)
	temp= (rrt_node *)NULL;
    }
  
  if (proche)
    {
      temp = nearest_neighbor((*node)->q, tree, dist_choice, (double *)NULL);
      (*nb_lm)++;
      /* test de collision */
      if(p3d_equal_config((p3d_rob *)p3d_get_desc_curid(P3D_ROBOT),
			  (*node)->q, temp->q))
	PrintInfo(("erreur %d\n", __LINE__));
      
      if (collide_rrt ((*node)->q, temp->q))
	{
	  *rnode1 = *node;
	  *rnode2 = temp;
	  link = TRUE;
	}
    }
  
  return (link);
}

