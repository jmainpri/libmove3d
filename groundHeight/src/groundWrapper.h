#ifndef GROUNDWRAPPER_H
#define GROUNDWRAPPER_H

#ifdef __cplusplus
extern "C"{
#endif

  int GHintersectionVerticalLineWithGround(void* Obj,double inX, double inY, double* groundHeight);
  void GHinitializeGrid(void* Obj);
  int GHaddVertex(void* Obj, double  inX,  double  inY,  double  inZ);
  int GHaddTriangle(void* Obj,unsigned int inPt1, unsigned int inPt2, unsigned int inPt3);
  void * GHinitCgroundHeight(unsigned int inNbTrianglesPerSquare);

#ifdef __cplusplus
}
#endif

#endif
