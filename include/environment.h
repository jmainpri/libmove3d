
#ifndef _ENVIRONMENT_H
#define _ENVIRONMENT_H

/* Structure d'environnement */
typedef struct env {
  char      *name;
  int       num;
  int       nof_objs; /* total number of objects, both obstacles and robot bodies */
  int       no;
  int       nr;
  p3d_obj   **o,*ocur;
  p3d_rob   **robot,    /* array of robots for multi-robot */
    *cur_robot;         /* pointer to current robot */
  p3d_box   box;
  int INIT; /* environnement initialise graphiquement ou pas */
  double dmax; /* pas de discretisation maximal */
  double object_tolerance; /* real object tolerance (to enlarge objects)*/
  double graphic_dmin; /* pas de discretisation graphique */
  /*  struct graph *graphPt; */
  int      modif; /* TRUE if environment recently modified, FALSE otherwise */
  int   envBoxIsUpToDate; /* SDK: TRUE if the environment box (box field in
			     this struct) was computed after last modification
			     of environment, FALSE otherwise */
  int   ccIsUpToDate;     /* SDK: TRUE if a collision checker was initialized
			     after last modification of environment,
			     FALSE otherwise */
  int nof_new_obsts;      /* number of added obstacles after latest
			     (re)initialization or update of collision checker */
  int *new_obsts;         /* array of length nof_new_obsts, for each new obstacle
			     since latest (re)initialization or update of collision
			     checker we store the ranknumber in list p3d_obj **o
			     for that obstacle */
  struct statistics * stat;  /* Global Environment statistics ; Commit Jim ; date: 01/10/2008 */
} p3d_env,*pp3d_env;

#endif /* #ifndef _ENVIRONMENT_H */


