#ifndef FLAT_STRUCT_H
#define FLAT_STRUCT_H

#include "const_flat.h"

/* data-structure used in flatness library */

/* geometric parameters of a trailer connection 
   imported from GenoM module genPos */
/* GENPOS_TRCART_KIN_PARAM_STR --> GENPOS_TRAILER_CONNECTION */
typedef struct GENPOS_TRAILER_CONNECTION {
  double l1; /* distance axe des roues robot, pt d'attache */ 
  double l2; /* distance axe des roues remorque, pt d'attache */
  double maxAngle; 
} GENPOS_TRAILER_CONNECTION;

/* Configuration pour forme plate */
typedef struct TR_FLAT_CONFIG_STR {
    double xp;
    double yp;
    double tau;
    double kappa;
} TR_FLAT_CONFIG_STR;

typedef struct FLAT_LOCAL_PATH_STR { /* un bout d'un chemin donne' par pl. local */
  TR_FLAT_CONFIG_STR initFlatConf;/* conf d'origine chemin incluant ce bout */
  TR_FLAT_CONFIG_STR finalFlatConf;    /* conf finale chemin incluant ce bout */
  double  v2;
  double velCoeff;/* dilatation du parametrage global par rapport au local */
  double alpha_0;
  double alpha_1;
  double  u_start;       /* indices extremes dans le parametrage du chemin */
  double  u_end;         /* incluant ce bout (entre 0 et 1)*/
} FLAT_LOCAL_PATH_STR;

typedef struct FLAT_STR {
  GENPOS_TRAILER_CONNECTION distAxleToAxis;
  double array_L[FLAT_NUMBER_L+1];   /*echantillonnage de L */
  double array_phi[FLAT_NUMBER_C];   /*echantillonnage de phi */
  double phi_max;         /* angle maximal entre robot et remorque */
  double maxCurvature;    /* courbure max dans la phase faisable */
} FLAT_STR;

typedef struct FLAT_TAB_LOCAL_PATH {
  int tabSize;
  int unused;
  FLAT_LOCAL_PATH_STR        tab[FLAT_MAX_NUMBER_LOCAL_PATH];
} FLAT_TAB_LOCAL_PATH;

#endif
