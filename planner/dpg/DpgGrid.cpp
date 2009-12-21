#include "../planner/dpg/proto/DpgGrid.h"

using namespace std;

DpgGrid::DpgGrid(p3d_env * env){
  _env = env;
  _nbMaxCells = 30;
  double cellSize = env->box.x2 - env->box.x1;
  cellSize = MAX(env->box.y2 - env->box.y1, cellSize);
  cellSize = MAX(env->box.z2 - env->box.z1, cellSize);
  cellSize /= _nbMaxCells;

  _originCorner[0] = env->box.x1;
  _originCorner[1] = env->box.y1;
  _originCorner[2] = env->box.z1;

  _nbCellsX = (env->box.x2 - env->box.x1)/cellSize;
  _nbCellsY = (env->box.y2 - env->box.y1)/cellSize;
  _nbCellsZ = (env->box.z2 - env->box.z1)/cellSize;

  _nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
  _cellSize[0]= _cellSize[1] = _cellSize[2] = cellSize;
  this->createAllCells();

  //Build the meshes env edges
  if(env->o[0]->pol[0]->poly->the_edges == NULL){
    for(int i = 0; i < env->no; i++){
      p3d_obj * obj = env->o[i];
      for(int j = 0; j < obj->np; j++){
        poly_build_edges(obj->pol[j]->poly);
      }
    }
  }
  init();
}

DpgGrid::~DpgGrid(){
  for(int i = 0; i < _nbCells; i++){
    if(getCell(i) != NULL){
      delete(getCell(i));
    }
  }
}

void DpgGrid::init(void){
  //unvalid static object Cells
  for(int i = 0; i < _env->no; i++){
    unvalidObjectCells(_env->o[i]);
  }
  //unvalid cells for each robot except current one ?
}

void DpgGrid::unvalidObjectCells(p3d_obj* obj){
  vector<DpgCell*> cellTab = getCellListForObject(obj);
  for(unsigned int i = 0; i < cellTab.size(); i++){
    cellTab[i]->setValid(false);
  }
}

vector<DpgCell*> DpgGrid::getCellListForObject(p3d_obj* obj){
  vector<DpgCell*> edgeCells, objectCells;
  vector<DpgCell*>::iterator it;

  for(int i = 0; i < obj->np; i++){
    if(obj->pol[i]->TYPE != P3D_GRAPHIC){
      p3d_polyhedre * poly = obj->pol[i]->poly;
      for(unsigned int j = 1; j <= poly->nb_faces; j++){
        //getCellListForEdge(poly, j, edgeCells);
        getCellListForFace(poly, j, edgeCells);
        //Add edge cell to object cells
        objectCells.insert(objectCells.end(), edgeCells.begin(), edgeCells.end());
      }
    }
  }
  return objectCells;
}

vector<DpgCell*> DpgGrid::getCellPoint(double *point, int dx = 0, int dy = 0, int dz = 0){
  vector<DpgCell*> pointCells;
  int x = 0, y = 0, z = 0;
  switch(dx){
    case -1:{
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      break;
    }
    case 1:{
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      break;
    }
  }
  switch(dy){
    case -1:{
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      break;
    }
    case 1:{
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      break;
    }
  }
  switch(dz){
    case -1:{
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
      break;
    }
    case 1:{
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
      break;
    }
  }
  if(dx == 0){//on a YZ plan
    if(dy == 0){//4 cells
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else if(dz == 0){//4cells
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else{//2cells
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      x = (int)floor((point[0]-_originCorner[0])/_cellSize[0]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }
  }else if(dy == 0){//on a XZ plan
    if(dz != 0){//4cells
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }else{//2cells
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]) - 1;
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
      y = (int)floor((point[1]-_originCorner[1])/_cellSize[1]);
      pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    }
  }else if(dz == 0){//on a XY plan
    z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]) - 1;
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
    z = (int)floor((point[2]-_originCorner[2])/_cellSize[2]);
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
  }else{//1cell
    pointCells.push_back(dynamic_cast<DpgCell*>(getCell(x, y, z)));
  }
  return pointCells;
}

void DpgGrid::draw(){
  for(int i=0; i < getNumberOfCells(); i++){
    DpgCell* cell = static_cast<DpgCell*>(getCell(i));
    if(!cell->isValid()){
      cell->draw();
    }
  }
  for(int i=0; i < getNumberOfCells(); i++){
    DpgCell* cell = static_cast<DpgCell*>(getCell(i));
    if(cell->isValid()){
      cell->draw();
    }
  }
}

//protected functions
void DpgGrid::getCellListForEdge(p3d_polyhedre * poly, int edgeId, vector<DpgCell*>& edgeCells){
  p3d_vector3 point = {0, 0, 0};
  p3d_vector3 point1 = {0, 0, 0};
  p3d_vector3 point2 = {0, 0, 0};
  //Recuperer les coordonnees des deux extremitees de l'arrete
  poly_get_point_in_edge(poly, edgeId, 1, &point[0], &point[1], &point[2]);
  p3d_xformVect(poly->pos, point, point1);
  poly_get_point_in_edge(poly, edgeId, 2, &point[0], &point[1], &point[2]);
  p3d_xformVect(poly->pos, point, point2);
  for(int i = 0; i < 3; i++){
    point1[i] += poly->pos[i][3];
    point2[i] += poly->pos[i][3];
  }
  getCellListForEdge(point1, point2, edgeCells);
}

void DpgGrid::getCellListForEdge(double * point1, double * point2, vector<DpgCell*>& edgeCells){
  //Cells for start point
  double p1p2[3] = {point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]};
  int dx = ABS(p1p2[0]) < EPS6 ? 0 : p1p2[0] > 0 ? 1 : -1;
  int dy = ABS(p1p2[1]) < EPS6 ? 0 : p1p2[1] > 0 ? 1 : -1;
  int dz = ABS(p1p2[2]) < EPS6 ? 0 : p1p2[2] > 0 ? 1 : -1;
//   vector<DpgCell*> edgeCells;

//Fast Voxel Transversal Algorithm
  

  
/*  vector<DpgCell*> pointCells = getCellPoint(point1, dx, dy, dz);
  edgeCells.insert(edgeCells.end(), pointCells.begin(), pointCells.end());
  //Cells for end point
  pointCells.clear();
  pointCells = getCellPoint(point1, -dx, -dy, -dz);
  edgeCells.insert(edgeCells.end(), pointCells.begin(), pointCells.end());*/
}

//  int    polySides  =  how many corners the polygon has
//  float  polyX[]    =  horizontal coordinates of corners
//  float  polyY[]    =  vertical coordinates of corners
//  float  x, y       =  point to be tested
//
//  The function will return YES if the point x,y is inside the polygon, or
//  NO if it is not.  If the point is exactly on the edge of the polygon,
//  then the function may return YES or NO.
//
//  Note that division by zero is avoided because the division is protected
//  by the "if" clause which surrounds it.
int DpgGrid::pointInPolygon(int nbPolyPoints, double* absys, double * ordon, double pointAbsys, double pointOrdon ) {
  int i, j, c = 0;
  for (i = 0, j = nbPolyPoints-1; i < nbPolyPoints; j = i++) {
    if ( ((ordon[i]>pointOrdon) != (ordon[j]>pointOrdon)) &&
   (pointAbsys < (absys[j]-absys[i]) * (pointOrdon-ordon[i]) / (ordon[j]-ordon[i]) + absys[i]) )
       c = !c;
  }
  return c;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param index the cell index
 * \param x The position of the cell over x
 * \param y The position of the cell over y
 * \param z The position of the cell over z
 */
DpgCell* DpgGrid::createNewCell(int index, int x, int y, int z )
{
    DpgCell* newCell = new DpgCell( index, computeCellCorner(x,y,z) , this );
    return newCell;
}

void DpgGrid::getCellListForFace(p3d_polyhedre * poly, int faceId, vector<DpgCell*>& edgeCells){
  int nbPolyPoints = poly_get_nb_points_in_face(poly, faceId);
  double pX[nbPolyPoints];
  double pY[nbPolyPoints];
  double pZ[nbPolyPoints];
  int minX = P3D_HUGE, maxX = -P3D_HUGE, minY = P3D_HUGE, maxY = -P3D_HUGE, minZ = P3D_HUGE, maxZ = -P3D_HUGE;
  for(int i = 0; i < nbPolyPoints; i++){
    poly_get_point_in_pos_in_face(poly, faceId, i + 1, &pX[i], &pY[i], &pZ[i]);
    p3d_vector3 point = {pX[i], pY[i], pZ[i]};
    p3d_vector3 point2 = {0, 0, 0};
    p3d_xformVect(poly->pos, point, point2);
    for(int j = 0; j < 3; j++){
      point2[j] += poly->pos[j][3];
    }
    pX[i] = point2[0];
    pY[i] = point2[1];
    pZ[i] = point2[2];
    minX = pX[i] < minX ? pX[i] : minX;
    maxX = pX[i] > maxX ? pX[i] : maxX;
    minY = pY[i] < minY ? pY[i] : minY;
    maxY = pY[i] > maxY ? pY[i] : maxY;
    minZ = pZ[i] < minZ ? pZ[i] : minZ;
    maxZ = pZ[i] > maxZ ? pZ[i] : maxZ;
  }
//plan XY
  Vector3d pos;
  pos[0] = minX;
  pos[1] = minY;
  pos[2] = _originCorner[2];
  
  DpgCell* iiCell = dynamic_cast<DpgCell*>(getCell(pos));
  pos[0] = maxX;
  DpgCell* aiCell = dynamic_cast<DpgCell*>(getCell(pos));
  pos[1] = maxY;
  DpgCell* aaCell = dynamic_cast<DpgCell*>(getCell(pos));
  pos[0] = minX;
  DpgCell* iaCell = dynamic_cast<DpgCell*>(getCell(pos));
  vector<DpgCell*> cellsToCheck;
  int nbXcells = aiCell->getIndex() - iiCell->getIndex();
  int nbYcells = (iaCell->getIndex() - iiCell->getIndex())/_nbCellsX;
  for(int i = 0, curIndex = iiCell->getIndex(); i <= nbYcells; i++, curIndex = iiCell->getIndex() + i * _nbCellsX){
    for(int j = 0; j <= nbXcells; j++, curIndex++){
      DpgCell* cell = dynamic_cast<DpgCell*>(getCell(curIndex));
      //For each cell check if one of the corners is in the poly
      Vector3d corner = cell->getCorner();
      double pAbsys = corner[0], pOrd = corner[1];
      for(int k = 0; k < 4; k++){
        switch(k){
          case 1:{
            pAbsys = corner[0] + _cellSize[0];
            break;
          }
          case 2:{
            pOrd = corner[1] + _cellSize[1];
            break;
          }
          case 3:{
            pAbsys = corner[0];
            break;
          }
        }
        if (pointInPolygon(nbPolyPoints, pX, pY, pAbsys, pOrd)){
          //the corner is inside the polygon: Select the cell and go to the next polygon
          edgeCells.push_back(cell);
          break;
        }
      }
    }
  }
//plan XZ
}
