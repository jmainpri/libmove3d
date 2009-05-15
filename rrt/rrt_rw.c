#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Rrt-pkg.h"

/* static void write_tree (FILE *fich, rrt_graph *graph); */
/* static void write_comp (rrt_node *node, int nb, FILE *fich); */
/* static int read_tree (FILE *fich, rrt_graph *graph); */
/* static void charge_tree(FILE *fich, rrt_graph *graph); */


/******************************************************/
/* test si on peut enregistrer l'arbre sur ce fichier */
/******************************************************/

void save_tree(char *file)
{
  FILE *fich;
  rrt_graph *g_rrt;

  g_rrt = get_rrt_graph();
  if ((fich = fopen(file, "r")) == NULL)
    {
      PrintInfo(("fichier cree\n"));
      fclose (fich);
      fich = fopen(file, "w");
      write_tree (fich, g_rrt);
      fclose (fich);
    }
  else
    {
      PrintInfo(("erreur: on ne peut pas ecrire dans ce fichier\n"));
    }
}


/***********************************************/
/* test si on peut lire l'arbre sur ce fichier */
/***********************************************/

void load_tree(char *file)
{
  FILE *fich;
  rrt_graph *g_rrt;
  int ko;

  g_rrt = get_rrt_graph();
  if ((fich = fopen(file, "r")) != NULL)
    {
      PrintInfo(("lecture fichier\n"));
      ko = read_tree (fich, g_rrt);
      if (ko)
	PrintInfo(("erreur de lecture\n"));
      fclose (fich);
    }
  else
    {
      PrintInfo(("erreur: on ne peut pas lire dans ce fichier\n"));
    } 
}


/*************************/
/* sauvegarde de l'arbre */
/*************************/

void
write_tree (FILE *fich, rrt_graph *graph)
{
  char *env_name;

  env_name = p3d_get_desc_curname(P3D_ENV);
  fprintf(fich,"p3d_env %s \n",env_name);
  fprintf(fich,"p3d_robot %s \n\n",graph->r->name);
  fprintf(fich,"p3d_nnodes %d \n",graph->nb_node);
  fprintf(fich,"p3d_nbq %d \n",graph->nb_node);
  fprintf(fich,"p3d_nbq_free 0 \n");
  fprintf(fich,"rrt_chemin %d %d %d\n",graph->chemin,graph->n1,graph->n2);
  fprintf(fich,"rrt_nb_node %d %d\n",graph->nb_node1,graph->nb_node2);
  fprintf(fich,"p3d_time %f \n",graph->time);
  fprintf(fich,"p3d_local_call %d \n",graph->nb_lm);
  fprintf(fich,"p3d_test_coll %d \n\n", graph->nb_col);

  write_comp (graph->init, 1, fich);
  write_comp (graph->goal, 2, fich);
}


/***************************************/
/* sauvegarde d'une composante connexe */
/***************************************/

void 
write_comp (rrt_node *node, int nb, FILE *fich)
{
  rrt_edge *t_edge;
  int i;
  int nb_dof = p3d_get_robot_ndof();                      /* number of dof */
  int plan = p3d_get_PLANNER_CHOICE();
  p3d_rob *r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);  /* current robot */
  int nb_edge;
  rrt_node *temp;

  fprintf(fich,"p3d_comp %d \n\n", nb);
  temp = node;
  while (temp != NULL)
    {
      fprintf(fich,"   p3d_node %d \n",temp->number);
      fprintf(fich,"      p3d_q");
      for (i=0; i<nb_dof; i++)
	fprintf(fich," %f",temp->q[i]);
      t_edge = temp->fedge;
      nb_edge = 0;
      while (t_edge != NULL) {
	nb_edge ++;
	t_edge = t_edge->next;
      }
      if (temp != node) {
	fprintf(fich,"\n      p3d_neighb %d", (nb_edge+1));
	fprintf(fich," %d",temp->precedge->n1->number);
      }
      else {
	fprintf(fich,"\n      p3d_neighb %d", nb_edge);
      }
      t_edge = temp->fedge;
      while (t_edge != NULL) {
	fprintf(fich," %d",t_edge->n2->number);
	t_edge = t_edge->next;
      }
      if (temp != node)
	fprintf(fich,"\n      p3d_nedge %d\n", (nb_edge+1));
      else
	fprintf(fich,"\n      p3d_nedge %d\n", nb_edge);
      if (temp != node) {
	fprintf(fich,"         p3d_edge %d %d %d %f\n",
		temp->precedge->n2->number,
		temp->precedge->n1->number, 
		plan, p3d_dist_q1_q2 (r, temp->precedge->n2->q, 
				      temp->precedge->n1->q));
      }
      t_edge = temp->fedge;
      while (t_edge != NULL) {
	fprintf(fich,"         p3d_edge %d %d %d %f\n",
		t_edge->n1->number,
		t_edge->n2->number, 
		plan, p3d_dist_q1_q2 (r, t_edge->n1->q, 
				      t_edge->n2->q));
	t_edge = t_edge->next;
      }
      fprintf(fich,"     p3d_end_node\n\n");
      temp = temp->next;
    }
  fprintf(fich,"p3d_end_comp \n\n");
}


/*************************/
/* chargement d'un arbre */
/*************************/

int read_tree (FILE *fich, rrt_graph *graph)
{
  char fct[100];
 char name[256];
  p3d_env *env;
  p3d_rob *r;
  int nombre;
  double d_nb;
  int counter = 0;
  rrt_node *n1, *n2;

  r = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  reset();

  while(fscanf(fich,"%s", fct) != EOF) {
  
    if (strcmp(fct,"p3d_env")==0) {
	counter ++;
	fscanf(fich, "%s", name);
	env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	if(strcmp(env->name,name)!= 0) {
	    PrintInfo(("environnement courant : %s ; environnement lu : %s\n",
                   env->name,name));
	    return(TRUE);
	}
	continue;
    }
    
    if (strcmp(fct,"p3d_robot")==0) {
      counter ++;
      fscanf(fich, "%s", name);
      if(strcmp(r->name,name)!= 0) {
	PrintInfo(("robot courant : %s ; robot lu : %s\n",r->name,name));
	return(TRUE);
      }
      continue;
    }

    if (strcmp(fct,"p3d_nnodes")==0) {
      counter ++;
      fscanf(fich, "%d", &nombre);
      graph = MY_ALLOC(rrt_graph,1);
      graph->init = NULL;
      graph->goal = NULL;
      graph->r = r;
      set_rrt_graph(graph);
      graph->nb_node = nombre;
      continue;
    }
    
    if (strcmp(fct,"rrt_chemin")==0) {
      counter ++;
      fscanf(fich, "%d", &nombre);
      graph->chemin = nombre;
      fscanf(fich, "%d", &nombre);
      graph->n1 = nombre;
      fscanf(fich, "%d", &nombre);
      graph->n2 = nombre;
      continue;
    }

    if (strcmp(fct,"rrt_nb_node")==0) {
      counter ++;
      fscanf(fich, "%d", &nombre);
      graph->nb_node1 = nombre;
      fscanf(fich, "%d", &nombre);
      graph->nb_node2 = nombre;
      continue;
    }

    if (strcmp(fct,"p3d_time")==0) {
      counter ++;
      fscanf(fich, "%lf", &d_nb);
      graph->time = d_nb;
      continue;
    }

    if (strcmp(fct,"p3d_local_call")==0) {
      counter ++;
      fscanf(fich, "%d", &nombre);
      graph->nb_lm = nombre;
      continue;
    }

    if (strcmp(fct,"p3d_test_coll")==0) {
      counter ++;
      fscanf(fich, "%d", &nombre);
      graph->nb_col = nombre;
    }
    
    if (counter == 8)
      break;
  }
  
  if (counter == 8) {
    rewind(fich);
    while(fscanf(fich,"%s", fct) != EOF) {
      if (strcmp(fct,"p3d_comp")==0) {
	fscanf (fich, "%d", &nombre);
	if (nombre == 1)
	  charge_tree(fich, graph);
	if (nombre == 2)
	  charge_tree(fich, graph);
	continue;
      }
    }
    if (graph->chemin) {
      n1 = graph->init;
      n2 = graph->goal;
      while ((n1->number != graph->n1) && (n1->number != graph->n2)) 
	n1 = n1->next;
      while ((n2->number != graph->n1) && (n2->number != graph->n2)) 
	n2 = n2->next;
      generate_curve(n1, n2);
//	  g3d_add_traj("rrt search",p3d_get_desc_number(P3D_TRAJ)); // reported in user interface (forms)
    }
    return (FALSE);
  }
  else {
    reset();
    return (TRUE);
  }
}

void charge_tree(FILE *fich, rrt_graph *graph)
{
  char fct[100];
  double *q;
  p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  int nb_dof = p3d_get_robot_ndof();                       /* number of dof */
  int i;
  double d_nb;
  int number;
  int neigh;
  int counter = 0;
  int node_init = FALSE;
  rrt_node *nearnode;
  rrt_node **lastnode;


  lastnode = MY_ALLOC(pt_rrtnode, 1);
  q = p3d_alloc_config(robotPt);

  while(fscanf(fich,"%s",fct) != EOF) {
   
    if (strcmp(fct,"p3d_node")==0) {
      counter ++;
      fscanf(fich,"%d",&number);
      continue;
    }
    
    if (strcmp(fct,"p3d_q")==0) {
      counter ++;
      for (i=0; i<nb_dof; i++) {
	fscanf(fich,"%lf",&d_nb);
	q[i] = d_nb;
      }
      continue;
    }
    
    if (strcmp(fct,"p3d_neighb")==0) {
      counter ++;
      fscanf(fich,"%d",&neigh);
      fscanf(fich,"%d",&neigh);
      continue;
    }

    if (counter == 3) {
      counter = 0;
      if (number == 1) {
	node_init = TRUE;
	/* initialisation de l'arbre, node init */
	graph->init = MY_ALLOC(rrt_node, 1);
	graph->init->q = p3d_alloc_config(robotPt);
	graph->init->fedge = NULL;
	graph->init->precedge = NULL;
	graph->init->prec = NULL;
	graph->init->next = NULL;
	for (i=0; i<nb_dof; i++)
	  graph->init->q[i]=q[i];
	graph->init->number = 1;
	(*lastnode) = graph->init;
      }
      if (number == 2) {
	node_init = FALSE;
	/* initialisation de l'arbre, node goal */
	graph->goal = MY_ALLOC(rrt_node, 1);
	graph->goal->q = p3d_alloc_config(robotPt);
	graph->goal->fedge = NULL;
	graph->goal->precedge = NULL;
	graph->goal->prec = NULL;
	graph->goal->next = NULL;
	for (i=0;i<nb_dof;i++)
	  graph->goal->q[i]=q[i];
	graph->goal->number = 2;
	(*lastnode) = graph->goal;
      }
      if ((number!=1) && (number!=2)) {
	if (node_init) {
	  nearnode = graph->init;
	}
	else {
	  nearnode = graph->goal;
	}
	while (neigh != nearnode->number)
	  nearnode = nearnode->next;
	add_rrt_node(nearnode, q, lastnode, number);
      }
      continue;
    }
    if (strcmp(fct,"p3d_end_comp")==0)
      break;
  }
  MY_FREE(lastnode,pt_rrtnode, 1);
  p3d_destroy_config(robotPt, q);
}

