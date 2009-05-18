/*
 * Copyright (c) 2002 LAAS/CNRS -- RIA --
 * Daniel SIDOBRE -- de'cembre 2006
 */
#ifndef GB_GENOM_H
#define GB_GENOM_H

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


#define GB_GENOM
#include "gbStruct.h"

#endif
