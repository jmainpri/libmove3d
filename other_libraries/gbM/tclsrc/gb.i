/*
 * Copyright (c) 2002 LAAS/CNRS -- RIA --
 * Daniel SIDOBRE -- mai 2002
 */
%module gb
%include "typemaps.i"

%{
#include "gbGENOM.h"
#include "gb.h"
#include "Proto_gb.h"
#include "Proto_gbModeles.h"

  inline void Gb_vitesse_plus(Gb_vitesse* a, Gb_vitesse* b, Gb_vitesse* s) {
    Gb_v6_plus((Gb_v6*) a, (Gb_v6*) b, (Gb_v6*) s);
  }

  inline void Gb_vitesse_moins(Gb_vitesse* a, Gb_vitesse* b, Gb_vitesse* s) {
    Gb_v6_moins((Gb_v6*) a, (Gb_v6*) b, (Gb_v6*) s);
  }

  inline double Gb_vitesse_module(const Gb_vitesse* a) {
    return Gb_v6_module((const Gb_v6*) a);
  }

  inline Gb_v3* Gb_vitesse_get_t(Gb_vitesse* v) {
    return Gb_v6_get_t((Gb_v6*) v);
  }

  inline Gb_v3* Gb_vitesse_get_r(Gb_vitesse* v) {
    return Gb_v6_get_r((Gb_v6*) v);
  }


  inline void Gb_force_plus(Gb_force* a, Gb_force* b, Gb_force* s) {
    Gb_v6_plus((Gb_v6*) a, (Gb_v6*) b, (Gb_v6*) s);
  }

  inline void Gb_force_moins(Gb_force* a, Gb_force* b, Gb_force* s) {
    Gb_v6_moins((Gb_v6*) a, (Gb_v6*) b, (Gb_v6*) s);
  }

  inline double Gb_force_module(const Gb_force* a) {
    return Gb_v6_module((Gb_v6*) a);
  }

  inline Gb_v3* Gb_force_get_t(Gb_force* v) {
    return Gb_v6_get_t((Gb_v6*) v);
  }

  inline Gb_v3* Gb_force_get_r(Gb_force* v) {
    return Gb_v6_get_r((Gb_v6*) v);
  }
%}

typedef struct Gb_v3 {
  double x;
  double y;
  double z;
} Gb_v3;

double Gb_v3_norme(const Gb_v3* e, Gb_v3* s);
double Gb_prs(const double w1[3],const double w2[3]);
extern void Gb_v3_cross_product(const Gb_v3 *u, const Gb_v3 *v, Gb_v3 *output);
extern double Gb_v3_product(const Gb_v3 *u, const Gb_v3 *v);
extern double Gb_v3_module(const Gb_v3 *u);
extern void Gb_v3_plus(const Gb_v3 *u, const Gb_v3 *v, Gb_v3 *output);
extern void Gb_v3_moins(const Gb_v3 *u, const Gb_v3 *v, Gb_v3 *output);
extern void Gb_v3_product_r(const Gb_v3 *u, double r, Gb_v3 *output);
extern void Gb_v3_div_r(const Gb_v3* u, double r, Gb_v3* output);

typedef struct Gb_v6 {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
} Gb_v6;

/* typedef Gb_v6 Gb_vitesse; */
/* typedef Gb_v6 Gb_force; */

typedef struct Gb_vitesse {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
} Gb_vitesse;

typedef struct Gb_force {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
} Gb_force;

typedef struct Gb_m33 {
  Gb_v3 vx;
  Gb_v3 vy;
  Gb_v3 vz;
} Gb_m33;

void Gb_m33_ppv(const Gb_v3* ve, Gb_m33* ms);

/* Comme on utilise les quaternions pour repr�senter des d�placements
 *  on garde la m�me philosophie que pour les d�placements bas�s sur
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

void Gb_dep_quat(const Gb_dep* dep, Gb_quat* q);
void Gb_quat_dep(const Gb_quat* q, Gb_dep* dep);
void Gb_quat_th(const Gb_quat* q, Gb_th* th);
void Gb_dep_th(const Gb_dep* dep, Gb_th* th);
void Gb_th_quat(const Gb_th* th, Gb_quat* q);
void Gb_th_dep(const Gb_th* th, Gb_dep* dep);

/*
 * On veut que Gb_v6_plus (moins, get_t et get_r) acceptent des Gb_v6,
 *  Gb_force et Gb_vitesse
 */

void Gb_v6_plus(const Gb_v6* a, const Gb_v6* b, Gb_v6* s);
void Gb_v6_moins(const Gb_v6* a, const Gb_v6* b, Gb_v6* s);
double Gb_v6_module(const Gb_v6* a);
Gb_v3* Gb_v6_get_t(Gb_v6* v);
Gb_v3* Gb_v6_get_r(Gb_v6* v);

void Gb_vitesse_plus(Gb_vitesse* a, Gb_vitesse* b, Gb_vitesse* s);
void Gb_vitesse_moins(Gb_vitesse* a, Gb_vitesse* b, Gb_vitesse* s);
double Gb_vitesse_module(const Gb_vitesse* a);
Gb_v3* Gb_vitesse_get_t(Gb_vitesse* v);
Gb_v3* Gb_vitesse_get_r(Gb_vitesse* v);
void Gb_force_plus(Gb_force* a, Gb_force* b, Gb_force* s);
void Gb_force_moins(Gb_force* a, Gb_force* b, Gb_force* s);
double Gb_force_module(const Gb_force* a);
Gb_v3* Gb_force_get_t(Gb_force* v);
Gb_v3* Gb_force_get_r(Gb_force* v);

typedef struct Gb_th {
  Gb_v3 vx;
  Gb_v3 vy;
  Gb_v3 vz;
  Gb_v3 vp;
} Gb_th;

void Gb_th_x_vitesse(const Gb_th* th, const Gb_vitesse* v, Gb_vitesse* vs);
void Gb_th_x_force(const Gb_th* th, const Gb_force* f, Gb_force* fs);
void Gb_th_produit(const Gb_th* a, const Gb_th* b, Gb_th* s);
void Gb_th_x_v3(const Gb_th* th, const Gb_v3* v, Gb_v3* vs);
void Gb_thInv_x_v3(const Gb_th* th, const Gb_v3* v, Gb_v3* vs);
void Gb_th_x_v6(const Gb_th* th, const Gb_v6* v, Gb_v6* vs);
void Gb_thInv_x_v6(const Gb_th* th, const Gb_v6* v, Gb_v6* vs);
void Gb_th_inverse(const Gb_th* th, Gb_th* ths);
void Gb_thInv_x_force(const Gb_th* th, const Gb_force* f, Gb_force* fs);
void Gb_quat_x_v3(const Gb_quat* q, const Gb_v3* v, Gb_v3* vs);
void Gb_quat_x_quat(const Gb_quat* q1, const Gb_quat* q2, Gb_quat* qs);
void Gb_quat_inverse(const Gb_quat* qi, Gb_quat* qo);
void Gb_quat_conjugue(const Gb_quat* qi, Gb_quat* qo);
void Gb_quat_interpole(const Gb_quat* q0, const Gb_quat* q1, double s,
		       Gb_quat* qo);
int Gb_quat_interpole_dep(const Gb_dep* d1, const Gb_dep* d2, double s,
			   Gb_dep* d_o);
void Gb_quat_interpole_dep2(const Gb_quat* q1, const Gb_quat* q2, double s,
			    Gb_quat* qo);
void Gb_quat_interpole_diff(const Gb_quat* q1, const Gb_quat* q2, double s,
			    Gb_quat* qq, Gb_quat* qd, Gb_quat* qdiff);

typedef struct Gb_dep {
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
  double a;
} Gb_dep;

void Gb_dep_set(Gb_dep* dep, double x, double y, double z,
		double rx, double ry, double rz, double a);

#%typemap(ignore) double*(double temp) "$1 = &temp;";
%typemap(in, numinputs=0) double*(double temp) "$1 = &temp;";
%typemap(argout)  double* x {
  Tcl_Obj* o;
  Tcl_ResetResult(interp);   /* reinitialisation */
  o = Tcl_NewDoubleObj((double) *($1));
  Tcl_ListObjAppendElement(interp, Tcl_GetObjResult(interp), o);
}
%typemap(argout)  double* y, double* z,
  double* rx, double* ry, double* rz, double* a {
  Tcl_Obj *o;
  o = Tcl_NewDoubleObj((double) *($1));
  Tcl_ListObjAppendElement(interp, Tcl_GetObjResult(interp), o);
}
void Gb_dep_get(Gb_dep* dep, double* x, double* y, double* z,
		double* rx, double* ry, double* rz, double* a);
#%typemap(ignore) double* ;
%typemap(in, numinputs=0) double* ;
%typemap(argout) double* ;

%include "gbModeles.i"

