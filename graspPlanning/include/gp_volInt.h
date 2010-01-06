
#ifndef VOLINT_H
#define VOLINT_H

/*
   ============================================================================
   macros
   ============================================================================
*/

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

/*
   ============================================================================
   constants
   ============================================================================
*/

#define MAX_VERTS 10000     /* maximum number of polyhedral vertices */
#define MAX_FACES 20000     /* maximum number of polyhedral faces */
#define MAX_POLYGON_SZ 3 /* maximum number of verts per polygonal face */

//#define X 0
//#define Y 1
//#define Z 2


/*
   ============================================================================
   data structures
   ============================================================================
*/

typedef struct {
  int numVerts;
  double norm[3];
  double w;
  int verts[MAX_POLYGON_SZ];
  struct VOLINT_POLYHEDRON *poly;
} VOLINT_FACE;

typedef struct VOLINT_POLYHEDRON {
  int numVerts, numFaces;
  double verts[MAX_VERTS][3];
  VOLINT_FACE faces[MAX_FACES];
} VOLINT_POLYHEDRON;


typedef struct Mass_properties{
  double mass, volume, density;
  double r[3];            /* center of mass */
  double J[3][3];         /* inertia tensor */
} Mass_properties;


#endif
