#ifndef _TRAJ_H
#define _TRAJ_H
/* Structure de trajectoire */
typedef struct traj {
  char       *name;       /* nom de la courbe */
  char       *file;       /* nom du fichier de la trajectoire */
  int        id;          /* identificateur unique de la trajectoire, 
                             set by p3d_end_traj() */ /* Carl */
  int        sdk_id;      /* identifier of trajectory in sdk. Might be the
			     same as id ... */
  int        num;         /* numero de trajectoire */
  int        nloc;        /* number of local paths */
  struct rob *rob;        /* robot */
  int        nlp;         /* number of local paths */
  double     range_param; /* range of parameter along the trajectory */
  struct localpath *courbePt;
  /* int        (*reed_shepp_free)(void *); */
} p3d_traj,*pp3d_traj;


#endif /* ifndef _TRAJ_H */
