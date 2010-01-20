#ifndef CONST_FLAT_H
#define CONST_FLAT_H

/* macros pour le calcul d'integrale */
#define EPS 1.0e-10
#define NEAR_ZERO 0.00000001
#define JMAX 40

/* calcul fonction L */
#define FLAT_NUMBER_L 4999    /* Nombre d'elements du vecteur_L */
#define NR_END 1
#define FREE_ARG char*

/* calcul del angle a partir de la courvure */
#define FLAT_NUMBER_C 4999   /* Nombre d'elements du vecteur_C */

/* ordre de derivation maximal dans la fonction combination */
#define MAX_DERIV_ORDER_IN_COMBINATION   5

/* taille maximale du tableau de chemin elementaires */
#define FLAT_MAX_NUMBER_LOCAL_PATH        200

#ifndef SIGN
#define SIGN(f)      (((f) < 0) ? (-1) : (1))
#endif

#endif
