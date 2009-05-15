#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Rrt-pkg.h"

/* les fonctions locales */
static int extend_lin_config (rrt_node *tree, 
		       double *q, double *qnear, 
		       int pas,int *nb_lm, double *qnew);

static int extend_lin_localpath (rrt_node *tree, 
		       double *q, double *qnear, 
		       int pas,int *nb_lm, double *qnew);

static int extend_lin_dec (rrt_node *tree, 
			   double *q, double *qnear, 
			   int pas, int *nb_lm, double *qnew);
static int extend_surface (rrt_node *tree, 
			   double *q, double *qnear, 
			   int *nb_lm, double *qnew);

static void new_state (double *q, double *qnear, 
		       double *qnew, double pas);

/*
============================
choice of extention 
============================
*/
/*
 * input: - *tree ...
 *        - *q the "goal" config (the random one),
 *        - *qnear the nearest in tree,
 *        - pas,
 *	  - *nb_lm, le nombre d'appel à la méthode locale.
 *	  - ext_choice the extension method,
 *	  - *qnew the new node ! (perhaps soon in tree).
 *      
 *  output: is qnew OK ?
*/
int extend (rrt_node *tree, double *q, double *qnear, 
	    int pas, int *nb_lm, int ext_choice, double *qnew)
{
  int newnode = 0;

  switch (ext_choice)
    {
    case EXTEND_LIN_CONFIG:
      newnode = extend_lin_config (tree, q, qnear, pas, nb_lm, qnew);
      break;
    case EXTEND_LIN_LOCALPATH:
      newnode = extend_lin_localpath (tree, q, qnear, pas, nb_lm, qnew);
      break;
    case EXTEND_LIN_DECREASE:
      newnode = extend_lin_dec (tree, q, qnear, pas, nb_lm, qnew);
      break;
    case EXTEND_SURFACE:
      newnode = extend_surface (tree, q, qnear, nb_lm, qnew);
      break;
    default:
      PrintInfo(("erreur dans le choix de la fonction extend\n"));
    } 
  return (newnode);
}



/****************************************/
/* extention linear of the tree *********/
/* return TRUE if added a new node ******/
/****************************************/
/*
  input: cf extend()...
*/
static int extend_lin_config (rrt_node *tree, double *q, double *qnear, 
		       int pas, int *nb_lm, double *qnew)
{
  int booleen = FALSE;   /* booleen confimant l'ajout du noeud dans l'arbre */
  int col = TRUE;        /* booleen testant la collision */
  rrt_graph *g_rrt = get_rrt_graph();	/* TODO G_RRT is global... */

  new_state (q, qnear, qnew, (double)pas);

  /* on verifie que la nouvelle position est possible */
  p3d_set_and_update_robot_conf(qnew);
  col = p3d_col_test();
  g_rrt->nb_col ++;                
  if (col == FALSE)
    {
      if (tree == g_rrt->init)	/* the init tree */
	{
	  if (collide_rrt (qnear, qnew))
	    booleen = TRUE;
	}
      else			/* the goal tree */
	{
	  if (collide_rrt (qnew, qnear))
	    booleen = TRUE;
	}
      (*nb_lm)++;
    }

  return (booleen);
}

/*
=============================
extend_lin_localpath
=============================
*/
/*
 * On prend pas% du chemin local.
 * input : - *tree, pour la direction d'extension,
 *         - *q, la config aléatoire,
 *         - *qnear, la configuration la plus "proche" de q
 *           dans l'arbre tree,
 *         - pas,
 *         - *nb_lm, le nb d'appel à la méthode locale,
 *         - *qnew, la nouvelle config (à ajouter à tree)
 * output : qnew OK ?
 */
static int extend_lin_localpath (rrt_node *tree, double *q, double *qnear, 
				 int pas, int *nb_lm, double *qnew)
{
  int booleen = FALSE;		/* booleen confimant l'ajout du noeud dans l'arbre */
  double u;			/* param on the localpath */
  int ntest = 0;
  double *q_new = NULL;

  pp3d_localpath c1 = (pp3d_localpath)NULL;
  pp3d_localpath c2 = (pp3d_localpath)NULL;

  rrt_graph *g_rrt = get_rrt_graph();	/* TODO G_RRT is global... */
  /* current robot */
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 

  /* création du chemin local (dans le bon sens) */
  if(tree == g_rrt->goal)
    {
      c1 = p3d_local_planner(robotPt, q, qnear);
      u = c1->range_param * (double)pas / 100.0;
      /* 10 % de c */
      q_new = c1->config_at_param(robotPt, c1, c1->range_param - u); 
      c2 = c1->extract_sub_localpath(robotPt, c1, c1->range_param - u, 
				     c1->range_param);
    }
  else if(tree == g_rrt->init)
    {
      c1 = p3d_local_planner(robotPt, qnear, q);
      u = c1->range_param * (double)pas / 100.0;
      q_new = c1->config_at_param(robotPt, c1, u); /* 10 % de c */
      c2 = c1->extract_sub_localpath(robotPt, c1, 0.0, u);
    }

  (*nb_lm)++;

  /* copy of q_new in qnew to use it */
  p3d_copy_config_into(robotPt, q_new, &qnew);
  
  /* qnew OK ? */
  p3d_set_and_update_robot_conf(qnew);
  booleen = !p3d_col_test();
  g_rrt->nb_col ++;                

  /* collision test in this trajectory */
  /* si la config qnew est valide */
  if(booleen)
    booleen = !p3d_col_test_localpath(robotPt, c2, &ntest);

  g_rrt->nb_col = g_rrt->nb_col + ntest;

  /* the end... */
  p3d_destroy_config(robotPt, q_new);

  /* destroy local path */
  c1->destroy(robotPt, c1);
  c2->destroy(robotPt, c2);

  return (booleen);
}

/****************************************/
/* extention linear of the tree *********/
/* return TRUE if added a new node ******/
/* decrease step if collision ***********/
/****************************************/

static int extend_lin_dec (rrt_node *tree, 
			   double *q, double *qnear, 
			   int pas, int *nb_lm, double *qnew)
{
  int booleen = FALSE;   /* booleen confimant l'ajout du noeud dans l'arbre */
  int col = TRUE;        /* booleen testant la collision */
  int nb_boucle;                  /* compteur du nombre de boucle */
  int continue_search = TRUE;     /* booleen pour arreter la boucle */
  double new_step;                /* nouveau pas calcule */
  rrt_graph *g_rrt;

  g_rrt = get_rrt_graph();

  (double)new_step = (double)pas;
  nb_boucle = 0;
  while (continue_search)
    {
      new_state (q, qnear, qnew, new_step);
      /* on verifie que la nouvelle position est possible */
      p3d_set_and_update_robot_conf(qnew);
      /* check if collision-free  config  */
      col = p3d_col_test();
      g_rrt->nb_col ++;                

      if (col == FALSE)
	{
 	  (*nb_lm)++;		/* dans collide_rrt */
	  if (tree == g_rrt->init)
	    {
	      if (collide_rrt (qnear, qnew))
		{
  		  booleen = TRUE;
		  return (booleen);
		}
	    }
	  else
	    {
	      if (collide_rrt (qnew, qnear))
		{
		  booleen = TRUE;
		  return (booleen);
		}
	    }
	}
      if (nb_boucle > 10)
	{
	  /* PrintInfo(("pas de nouvelle config\n")); */
	  continue_search = FALSE;
	}
      nb_boucle ++;
      (double)new_step = (double)new_step/(double)2.0;
    }
  return (booleen);
}


/*******************************************/
/* extention jusqu'aux surfaces de l'arbre */
/*******************************************/
/*
 *  input: - tree,
 *         - *q the random one (the goal),
 *         - *qnear the nearest rrt_node in tree (the start),
 *         - *nb_lm,
 *         - *qnew the new rrt_node to add in tree.
 *
 *  output: *qnew OK ?
 */

static int extend_surface (rrt_node *tree, double *q, double *qnear, 
			  int *nb_lm, double *qnew)
{
  /* current robot */
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT); 
  pp3d_localpath c = (pp3d_localpath)NULL;
  rrt_graph *g_rrt = get_rrt_graph();
  int njnt = p3d_get_robot_njnt();
  int i;
  double *distances, du;
  configPt q_new;

  int the_end=FALSE;		/* la fin du parcourt de c */
  double u = 0.;		/* un param de c */
  int ntest = 0;

  distances = MY_ALLOC(double, njnt+1);

  /* par default : */
  p3d_copy_config_into(robotPt,  qnear, &qnew);
  
  /* generate the local curve between the 2 configs */
  if(tree == g_rrt->goal)
    {
      c = p3d_local_planner(robotPt, q, qnear);
      u = c->range_param;
    }
  else if(tree == g_rrt->init)
    {
      c = p3d_local_planner(robotPt, qnear, q);
      u = 0.0;
    }
  (*nb_lm)++;
  
  p3d_set_and_update_robot_conf(qnear);
  
  /* there is no traj... */
  if(c == (pp3d_localpath)NULL)
    {
      c->destroy(robotPt, c);		/* TODO all is destroy ??? */  
      MY_FREE(distances, double, njnt+1);
      PrintError(("extend_surface : localpath not found\n"));
      /* TODO and now ??? */
    }
  
  /* peut-être la traj est sans coll ! */
  if (!p3d_col_test_localpath(robotPt, c, &ntest))
    {
      the_end = TRUE;
      p3d_copy_config_into(robotPt, q, &qnew);
    }
  g_rrt->nb_col = g_rrt->nb_col + ntest;

  while(the_end == FALSE)
    {
      /* compute the distance between bounding boxes */
      /* of robot bodies and obstacles */
      p3d_BB_dist_robot(robotPt, distances);
      for (i=0; i<=njnt; i++)
	if (distances[i] < p3d_get_env_dmax())
	  distances[i] = p3d_get_env_dmax();
      
      if(tree == g_rrt->goal)
	{
	  du = c->stay_within_dist(robotPt, c, u, BACKWARD, distances);
	  u = u - du;
	}
      else if(tree == g_rrt->init)
	{
	  du = c->stay_within_dist(robotPt, c, u, FORWARD, distances);
	  u = u + du;
	}

      q_new = c->config_at_param(robotPt, c, u);
      
      /* test coll */
      p3d_set_and_update_robot_conf(q_new);
      if(p3d_col_test())
	{
	  the_end = TRUE;
	}
      else
	{
	  /* copy of q_new in qnew to use it */
	  p3d_copy_config_into(robotPt, q_new, &qnew);
	}
      p3d_destroy_config(robotPt, q_new);
    }

  if(tree == g_rrt->goal)
    {
      q_new = c->config_at_param(robotPt, c, u*0.80);
    }
  else if(tree == g_rrt->init)
    {
      q_new = c->config_at_param(robotPt, c, u*0.80);
    }

  c->destroy(robotPt, c);		/* TODO all is destroy ??? */  

  MY_FREE(distances, double, njnt+1);

  /* les cas d'échec ... */
  p3d_set_and_update_robot_conf(qnew);  
  return(!p3d_col_test());
}



/************************/
/* generate a new config */
/************************/

static void new_state (double *q, double *qnear, double *qnew, double pas)
{
   int nb_dof = p3d_get_robot_ndof();                /* number of dof */
   int i;
   double step;

   step = ((double)pas)/100.0;

   /* generate the new config */
   for (i=0; i< nb_dof; i++)
       qnew[i] = qnear[i]+((q[i]-qnear[i])*step); 
}

