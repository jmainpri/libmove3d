#include "groundWrapper.h"
#include "groundHeight.h"

int GHaddVertex(void* Obj, double  inX,  double  inY,  double  inZ) { 
  return ((CgroundHeight*)Obj)->addVertex( inX,   inY,   inZ);
}

int GHaddTriangle(void* Obj,unsigned int inPt1, unsigned int inPt2, unsigned int inPt3)
{
  return ((CgroundHeight*)Obj)->addTriangle(inPt1,  inPt2, inPt3);
}

int GHintersectionVerticalLineWithGround(void* Obj,double inX, double inY, double* groundHeight)
{
  return ((CgroundHeight*)Obj)->intersectionVerticalLineWithGround(inX, inY, *groundHeight);
}

void GHinitializeGrid(void* Obj) {
  ((CgroundHeight*)Obj)->initializeGrid();
}


void * GHinitCgroundHeight(unsigned int inNbTrianglesPerSquare) {
  return new CgroundHeight(inNbTrianglesPerSquare);
}
