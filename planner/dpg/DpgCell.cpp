#include "../planner/dpg/proto/DpgCell.h"
#include "Graphic-pkg.h"

using namespace std;

DpgCell::DpgCell(int i, Vector3d corner, API::Grid* grid): API::Cell(i, corner, grid){
  _valid = 1;
  _edges.clear();
  _nodes.clear();
  _cellSize = grid->getCellSize();
  _visited = 0;
}
void DpgCell::draw(){
  Vector3d corner = this->getCorner();
  double xmin = corner[0], xmax = corner[0] + _cellSize[0], ymin = corner[1], ymax = corner[1] + _cellSize[1], zmin = corner[2], zmax = corner[2] + _cellSize[2];
  GLfloat mat_ambient_diffuse[4] = { 0., .0, .0, 0.5 };
  if(this->_valid){
    mat_ambient_diffuse[2] = 1;
    glLineWidth(1);
  }else{
    mat_ambient_diffuse[1] = 1;
    glLineWidth(2);
  }
  glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,mat_ambient_diffuse);
  glPushAttrib(GL_LINE_BIT);

  
  glBegin(GL_LINES);
  glVertex3f(xmin, ymin, zmin);
  glVertex3f(xmin, ymin, zmax);

  glVertex3f(xmax, ymin, zmin);
  glVertex3f(xmax, ymin, zmax);

  glVertex3f(xmax,  ymax, zmin);
  glVertex3f(xmax,  ymax, zmax);

  glVertex3f(xmin, ymax, zmin);
  glVertex3f(xmin, ymax, zmax);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(xmin, ymin, zmin);
  glVertex3f(xmax, ymin, zmin);
  glVertex3f(xmax, ymax, zmin);
  glVertex3f(xmin, ymax, zmin);
  glEnd();

  glBegin(GL_LINE_LOOP);
  glVertex3f(xmin, ymin, zmax);
  glVertex3f(xmax, ymin, zmax);
  glVertex3f(xmax, ymax, zmax);
  glVertex3f(xmin, ymax, zmax);
  glEnd();

  glPopAttrib();
}