/*
 * Copyright (c) 2002 LAAS/CNRS -- RIA --
 * Daniel SIDOBRE -- mai 2002
 */
#ifndef gbStruct_h
#define gbStruct_h

typedef struct Gb_v3 {
  double x;
  double y;
  double z;
} Gb_v3;


typedef struct Gb_v6 {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
} Gb_v6;

#ifdef GB_GENOM
#else
typedef Gb_v6 Gb_vitesse;
typedef Gb_v6 Gb_force;
#endif

typedef struct Gb_th {
  Gb_v3 vx;
  Gb_v3 vy;
  Gb_v3 vz;
  Gb_v3 vp;
} Gb_th;

typedef struct Gb_dep {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
  double a;
} Gb_dep;


typedef struct Gb_m33 {
  Gb_v3 vx;
  Gb_v3 vy;
  Gb_v3 vz;
} Gb_m33;

/* Comme on utilise les quaternions pour représenter des déplacements
 *  on garde la même philosophie que pour les déplacements basés sur
 *  un angle et une direction.
 */
typedef struct Gb_quat {
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  double w;
} Gb_quat;

/*
 *  bras 6r
 */

typedef struct Gb_6rParameters {
  double a2, r4;
  double epsilon;  /* to determine if the position is singular */
  double of1;
  double of2;
  double of3;
  double of4;
  double of5;
  double of6;
} Gb_6rParameters;

typedef struct Gb_q6 {
  double q1;
  double q2;
  double q3;
  double q4;
  double q5;
  double q6;
} Gb_q6;

typedef struct Gb_dataMGD {
  double d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12;
  double c1, c2, c3, c4, c5, c6;
  double s1, s2, s3, s4, s5, s6;
  double c23, s23;
  int e1, e2, e3;
  int inutile; /* pb alignement */
} Gb_dataMGD;

typedef enum Gb_statusMGI {
  MGI_OK = 0,
  MGI_ERROR,
  MGI_APPROXIMATE,
  MGI_SINGULAR
} Gb_statusMGI;

/*
 *  Pas vraiement indispensable, mais assure la coherance
 */
typedef struct Gb_jac {
  Gb_v6 c1;
  Gb_v6 c2;
  Gb_v6 c3;
  Gb_v6 c4;
  Gb_v6 c5;
  Gb_v6 c6;
} Gb_jac;

#endif
