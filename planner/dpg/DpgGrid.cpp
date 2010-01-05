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
        getCellListForEdge(poly, j, edgeCells);
        //getCellListForFace(poly, j, edgeCells);
        //Add edge cell to object cells
        //objectCells.insert(objectCells.end(), edgeCells.begin(), edgeCells.end());
      }
    }
  }
  //return objectCells;
  for(unsigned int i = 0; i < edgeCells.size(); i++){
    edgeCells[i]->setVisited(false);
  }
  return edgeCells;
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
  p3d_vector3 p1p2 = {point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]};

//Fast Voxel Transversal Algorithm
//Initialisation:  
  int step[3];
  for(int i = 0; i < 3; i++){
    step[i] = ABS(p1p2[i]) < EPS6 ? 0 : p1p2[i] > 0 ? 1 : -1;
  }
//Get the cell of the two points
  DpgCell* point1Cell = dynamic_cast<DpgCell*>(getCell(point1));
  DpgCell* point2Cell = dynamic_cast<DpgCell*>(getCell(point2));
  //If point1 and point2 are in the same cell, add it and go ahead
  if (point1Cell == point2Cell){
    if(!point1Cell->isVisited()){
      point1Cell->setVisited(true);
      edgeCells.push_back(point1Cell);
    }
    return;
  }
  Vector3d tMax(-1, -1, -1);
  Vector3d tDelta(_cellSize[0], _cellSize[1], _cellSize[2]);
  Vector3d p1CellCorner = point1Cell->getCorner();
  double edgeLenght =  sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1] + p1p2[2]*p1p2[2]);
//Compute tMax : the distance between the point1 and the first plan parralel to YZ, XZ and XY
  //Determine the closest plan to point 1
  for(int i = 0; i < 3; i++){
    if(step[i] != 0){
    p3d_vector3 point3 = {p1CellCorner[0], p1CellCorner[1], p1CellCorner[2]};
      if (step[i] > 0) {
        point3[i] += _cellSize[i];
      }
      p3d_vector3 p2p3 = {point3[0] - point2[0], point3[1] - point2[1], point3[2] - point2[2]};
      p3d_vector3 normale = {0, 0, 0};
      normale[i] = 1;
      double intersection = p3d_vectDotProd(normale, p2p3) / p3d_vectDotProd(normale,p1p2); //valeure en fonction de la taille du segment si 0 <intersection < 1 => le point d'intersection est au millieu du segment
      tMax[i] = edgeLenght * (1 - ABS(intersection));
      //compute tDelta
      if (step[i] > 0) {
        point3[i] -= _cellSize[i];
      }else {
        point3[i] += _cellSize[i];
      }
      p2p3[i] = point3[i] - point2[i];
      intersection = p3d_vectDotProd(normale, p2p3) / p3d_vectDotProd(normale,p1p2); //valeure en fonction de la taille du segment si 0 <intersection < 1 => le point d'intersection est au millieu du segment
      tDelta[i] = edgeLenght * ABS(1 - ABS(intersection)) + tMax[i];
    }else{ // parallel to X axis
      tMax[i] = P3D_HUGE;
    }    
  }
//Get the cells
  Vector3d index = getCoordinates(point1Cell);
  if(!point1Cell->isVisited()){
    point1Cell->setVisited(true);
    edgeCells.push_back(point1Cell);
  }
  DpgCell* selectedCell;
  do {
    if(tMax[0] < tMax[1]) {
      if(tMax[0] < tMax[2]) {
        index[0] += step[0];
        if(index[0] < 0 || index[0] >= _nbCellsX){
          printf("Out of bounds: Algo Error\n");
          return;
        }
        tMax[0] += tDelta[0];
      }else {
        index[2] += step[2];
        if(index[2] < 0 || index[2] >= _nbCellsZ){
          printf("Out of bounds: Algo Error\n");
          return;
        }
        tMax[2] += tDelta[2];
      } 
    }else {
      if(tMax[1] < tMax[2]) {
        index[1] += step[1];
        if(index[1] < 0 || index[1] >= _nbCellsY){
          printf("Out of bounds: Algo Error\n");
          return;
        }
        tMax[1] += tDelta[1];
      }else {
        index[2] += step[2];
        if(index[2] < 0 || index[2] >= _nbCellsZ){
          printf("Out of bounds: Algo Error\n");
          return;
        }
        tMax[2] += tDelta[2];
      }
    }
    selectedCell = dynamic_cast<DpgCell*>(getCell(index[0], index[1], index[2]));
    printf("Cell1 = %d, Cell2 = %d, Selected = %d\n", point1Cell->getIndex(), point2Cell->getIndex(), selectedCell->getIndex());
    if(!selectedCell->isVisited()){
      selectedCell->setVisited(true);
      edgeCells.push_back(selectedCell);
    }
  }while(selectedCell != point2Cell);
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

//Il Y'a un bug : Quand un polygone ne touche aucun coin d'une case, elle n'est pas retenue.
//TODO afaire pour le plan Y Z et fusionner les donnees

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
