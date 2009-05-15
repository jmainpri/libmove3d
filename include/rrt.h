/****************************************/
/* Fichier de structures pour les rrt   */
/****************************************/


#ifndef _RRT_H
#define _RRT_H
/* queques constantes */
#define EXTEND_LIN_CONFIG 1	/* extension lineaire dans l'espace
				   des configuration du robot */
#define EXTEND_LIN_LOCALPATH 2	/* extension lineaire le long */
				/* du chemin local */
#define EXTEND_LIN_DECREASE 3	/* extension lineaire à pas décroissant */
#define EXTEND_SURFACE 4	/* extension jusqu'au surface */

/*************************************/
/* definition structure node et edge */
/*************************************/

typedef struct rrtedge {
  struct rrtnode *n1;            /* node attached to this edge */
  struct rrtnode *n2;            /* node attached to this edge */
  struct rrtedge *prec;          /* previous edge in the list */
  struct rrtedge *next;          /* next edge in the list */
}rrt_edge;

typedef struct rrtnode {
  int number;                   /* number of the node */
  double *q;                    /* config in this part of list */
  struct rrtedge *fedge;        /* first edge of the list linked to this edge */
  struct rrtedge *precedge;     /* previous edge linked */
  struct rrtnode *prec;         /* previous node in the tree */
  struct rrtnode *next;         /* next node in the tree */
}rrt_node, *pt_rrtnode;

/*******************************/
/* definition structure graphe */
/*******************************/

typedef struct rrtgraph {
  p3d_rob *r;                /* current robot */
  rrt_node *init;            /* initial node */
  rrt_node *goal;            /* goal node */
  double time;               /* cpu time */
  int nb_node1;              /* number of node in init tree */            
  int nb_node2;              /* number of node in goal tree */            
  int nb_lm;                 /* number local method calls */
  int nb_node;               /* number of node */            
  int nb_col;                /* number of collision checker calls */
  int chemin;                /* find path */
  int n1;                    /* path link with this node number */
  int n2;                    /* path link with this other node number */
}rrt_graph;



/**********************************************************************/
/* definition structure pile de config, utiliser pour generer la traj */
/**********************************************************************/

typedef struct qpile{
  double *q;                     /* config */
  struct qpile *next;            /* next config in the list */
  struct qpile *prec;            /* previous config in the list */
}q_pile;

#endif
