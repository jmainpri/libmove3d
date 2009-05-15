#ifndef _RS_DEF_H
#define _RS_DEF_H

/*#include "p3d_sys.h" */

#define EPS1 0.1
#define EPS2 0.01
#define EPS3 0.001
#define EPS4 0.0001
#define EPS5 0.00001
#define EPS6 0.000001

#define EPSILON 1e-10

#define infini 10000000.

#define RS_ALL 0
#define RS_NO_CUSP -1
#define RS_WITH_CUSP -2
#define RS_DUBINS -3


/*******************************************************************/

typedef struct Stconfig{
  double x;
  double y;
  double z;
  struct Stconfig *suiv;
} Stconfig , *pStconfig;

/*******************************************************************
  CETTE STRUCTURE SERT A DEFINIR LA COURBE MIN PAR REED&SHEPP
  QUI EST DU TYPE C C S C C ||. UN ELEMENT DU TYPE Stcourbe SERA
  SOIT UN C SOIT UN S.
  ON AURA DONC AU MAX UNE LISTE DE 5 element Stcourbe
******************************************************************/
typedef struct Stcourbe{
  int type;               /* type = 1/2/3 pour droite/gauche/segment */
  Stconfig cd;            /* config init sur la courbe   */
  Stconfig cf;            /* config fin  sur la courbe   */
  double centre_x;        /* centre du cercle correspondant */
  double centre_y;        /* en x et y si c'est le cas (sinon 0) */
  double r;               /* rayon du disque */
  int sens;               /* sens = 1/-1 pour avant/arriere  */
  double val;             /* valeur de la portion de courbe */
  struct Stcourbe *suiv;  /* pointeur sur portion suivante  */
  struct Stcourbe *prec;  /* pointeur sur portion precedente */
  int  valid;             /* generalement TRUE quand pas de collision 
			     de la portion de trajectoire... NULL si 
			     la subdivision dans p2d_holo a ete stoppee pour
			     eviter un decoupage en portions minuscules */
} Stcourbe , *pStcourbe;

/* #include "rs_proto.h" */
/* #include "../localpath/proto/rs_dist_proto.h" */
/* #include "../localpath/proto/rs_curve_proto.h" */
/* #include "rs_inter_proto.h" */
/* #include "rs_interbm_proto.h" */
/* #include "rs_holo_proto.h" */
/* #include "rs_optim_proto.h" */
/* #include "p2d_rs_proto.h" */
#endif
