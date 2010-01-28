/*
 * Copyright (c) 2002 LAAS/CNRS -- RIA --
 * Xavier BROQUERE -- july 2009
 */
#ifndef gbManip_h
#define gbManip_h


/* nx  ox  ax  pz
    ny  oy  ay  py
    nz  oz  az  pz  */
typedef struct Gb_thMat {
  double nx, ny, nz;
  double ox, oy, oz;
  double ax, ay, az;
  double px, py, pz;
}  Gb_thMat;

typedef struct Gb_motionMat {
  double px, py, pz ;     /* Position */
  double qn, qi, qj, qk;  /* Orientation */
  double vx, vy, vz;      /* Linear velocity */
  double wx, wy, wz;      /* Angular velocity */
} Gb_motionMat;

typedef struct Gb_point2D {
  double x, y;
} Gb_point2D;

typedef struct Gb_point3D {
  double x, y, z;
} Gb_point3D;

typedef struct Gb_euler {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
}  Gb_euler;

typedef struct Gb_objectPose {
  int visible;
  Gb_thMat th;
} Gb_objectPose;

typedef struct Gb_objectMotion {
  int visible;
  Gb_motionMat motion;
} Gb_objectMotion;

#endif
