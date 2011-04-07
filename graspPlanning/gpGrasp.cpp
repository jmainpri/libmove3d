
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "GraspPlanning-pkg.h"
#include <math.h>
#include <string>
#include <sstream>


//! Default constructor of the class gpGrasp
gpGrasp::gpGrasp()
{
  autoGen         = true;
  ID              = 0;
  stability       = 0;
  IKscore         = 0;
  visibility      = 0;
  quality         = 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, frame);
  handID          = 0;
  object          = NULL;
  object_name     = "none";
  hand_type       = GP_HAND_NONE;
  tested          = false;
}

gpGrasp::gpGrasp(const gpGrasp &grasp)
{
  autoGen= grasp.autoGen;
  ID= grasp.ID;
  stability= grasp.stability;
  IKscore= grasp.IKscore;
  visibility= grasp.visibility;
  quality= grasp.quality;
  handID= grasp.handID;

  p3d_mat4Copy(grasp.frame, frame);

  object= grasp.object;
  object_name= grasp.object_name;
  hand_type= grasp.hand_type;
  tested= grasp.tested;

  contacts= grasp.contacts;
  config= grasp.config;  
  openConfig= grasp.openConfig;
  recomPlacement= grasp.recomPlacement;
}

gpGrasp::gpGrasp(const gpHand_properties &handProp)
{
  autoGen         = true;
  ID              = 0;
  stability       = 0;
  IKscore         = 0;
  visibility      = 0;
  quality         = 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, frame);
  handID          = 0;
  object          = NULL;
  object_name     = "none";
  hand_type       = handProp.type;
  tested          = false;
  
  config.resize(handProp.nb_fingers);
  openConfig.resize(handProp.nb_fingers);
}

gpGrasp::~gpGrasp()
{
  contacts.clear();
  config.clear();
}

//! Copy operator of the class gpGrasp.
gpGrasp & gpGrasp::operator = (const gpGrasp &grasp)
{
  if( this!=&grasp )
  {
    autoGen= grasp.autoGen;
    ID= grasp.ID;
    stability= grasp.stability;
    IKscore= grasp.IKscore;
    visibility= grasp.visibility;
    quality= grasp.quality;
    handID= grasp.handID;

    p3d_mat4Copy(grasp.frame, frame);

    object= grasp.object;
    object_name= grasp.object_name;
    hand_type= grasp.hand_type;
    tested= grasp.tested;

    contacts= grasp.contacts;
    config= grasp.config;  
    openConfig= grasp.openConfig;
    recomPlacement= grasp.recomPlacement;
  }

  return *this;
}


bool gpGrasp::operator == (const gpGrasp &grasp)
{
  return ID==grasp.ID;
}


//! Draws all the contacts of a grasp.
//! \param length lenght of each friction cone to draw
//! \param nb_slices number of segments of each cone discretization
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::draw(double length, int nb_slices)
{  
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i;
  p3d_vector3 p;
  p3d_matrix4 pose;
  GLfloat matGL[16];
  gpHand_properties hand;

  if(object!=NULL)
  {   p3d_get_freeflyer_pose(object, pose);  }
  else
  {   p3d_mat4Copy(p3d_mat4IDENTITY, pose);   }

  for(i=0; i<3; ++i)
  {
    p[i]= recomPlacement.center[i]-4*length*recomPlacement.plane.normale[i];
  }

  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
  glPushMatrix();
    glMultMatrixf(matGL);

    for(i=0; i<contacts.size(); ++i)
    { //if(i!=1) continue;
      if(i==0) { glColor3f(1.0, 0.0, 0.0);  }
      if(i==1) { glColor3f(0.0, 1.0, 0.0);  }
      if(i==2) { glColor3f(0.0, 0.0, 1.0);  }
      if(i==3) glColor3f(1.0, 1.0, 0.0);
      contacts[i].draw(length, nb_slices);
    }

    // draw recommanded placement:
    glLineWidth(4);
    glColor3f(0, 0, 1);
    glBegin(GL_LINE_LOOP);
    for(i=0; i<recomPlacement.contacts.size(); i++)
    {  glVertex3dv(recomPlacement.contacts[i].position);  }
    glEnd();

    glColor3f(1, 0, 1);
    glBegin(GL_LINES);
      glVertex3dv(recomPlacement.center);
      glVertex3dv(p);
    glEnd();

//     if(hand_type==GP_SAHAND_RIGHT)
//     { 
//       hand.initialize(GP_SAHAND_RIGHT);
//       p3d_mat4Mult(frame, hand.Tgrasp_frame_hand, Twrist);
//       q[0]= config.at(0);
//       for(i=0; i<4; ++i)
//       {
//         q[1]= config.at(3*i+1);
//         q[2]= config.at(3*i+2);
//         q[3]= config.at(3*i+3);
//         gpSAHfinger_forward_kinematics(Twrist, hand, q, p, fingerpad_normal, i+1);
//      
//         g3d_drawColorSphere(p[0], p[1], p[2], 0.008, Yellow, NULL);
//         g3d_drawOneLine(p[0], p[1], p[2], p[0]+0.05*fingerpad_normal[0], p[1]+0.05*fingerpad_normal[1], p[2]+0.05*fingerpad_normal[2], Yellow, NULL);
// //         for(unsigned int j=0; j<contacts.size(); ++j)
// //         {
// //           if( contacts[j].fingerID==(i+1) )
// //           {
// //             printf(" [%f %f %f]\n",contacts[j].normal[0], contacts[j].normal[1], contacts[j].normal[2]);
// //             printf("dot= %f\n",p3d_vectDotProd(contacts[j].normal,fingerpad_normal));
// //           }
// //           
// //         }
//       }
//     }

  glPopMatrix();
  glPopAttrib();
  
  return GP_OK;
}


//! Computes a cost for the given grasping configuration.
//! The biggest it is, the better it is.
//! \return the computed configuration cost 
double gpGrasp::configCost()
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::configCost(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  switch(hand_type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER:
       // for the gripper, we consider that is better to have a configuration
       // where the gripper is not close to its maximal opening so
       // we use a function that decreases with the config parameter but is finite in 0 
      return pow(1.2, config.at(0));
    break;
    case GP_SAHAND_RIGHT:
      return 0.0;
    break;
    case GP_SAHAND_LEFT:
      return 0.0;
    break;
    default:
       printf("%s: %d: gpGrasp::configCost(): unimplemented or unknown hand type.\n", __FILE__, __LINE__);
      return 0;
    break;
  }

}


//! Computes the distance between two grasps.
//! This is based on the distance between their two frames.
//! For now, it is the euclidean distance between the two frame centers.
//! \param grasp1 the first grasp
//! \param grasp2 the second grasp
//! \return the computed distance 
double gpGraspDistance(const gpGrasp &grasp1, const gpGrasp &grasp2)
{
  double d;//, alpha, beta;
  p3d_vector3 pos1, pos2, diff;//, cmass;
//   p3d_vector3 ray1, ray2, direction1, direction2;

  p3d_mat4ExtractTrans((p3d_matrix_type(*)[4]) grasp1.frame, pos1);
  p3d_mat4ExtractTrans((p3d_matrix_type(*)[4]) grasp2.frame, pos2);

  p3d_vectSub(pos1, pos2, diff);
  d= p3d_vectNorm(diff);
  return d;
}


//! WIP 
//! Computes a value trying to give a measure of the similarity between two grasps.
double gpGrasp::similarity(const gpGrasp &grasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::similarity(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return 0;
  }

  if(hand_type!=grasp.hand_type)
  {
    return 0;
  }

  unsigned int i, j;
  unsigned int nbCommonFingers;
  double dist;
  p3d_vector3 d;

  dist= 0;
  nbCommonFingers= 0;
  for(i=0; i<contacts.size(); ++i)
  {
    for(j=0; j<grasp.contacts.size(); ++j)
    {
      if(contacts[i].fingerID==grasp.contacts[j].fingerID)
      {
         p3d_vectSub(contacts[i].position, (p3d_matrix_type(*)) grasp.contacts[j].position, d);
         dist+= p3d_vectNorm(d);
         nbCommonFingers++;
      }
    }
  }

  if(nbCommonFingers==0)
  {  return 0; }

  return dist/((double) nbCommonFingers);

}

//! Computes the centroid of the polyhedron defined by the contact points of the grasp.
//! \param centroid the computed centroid of the contact polyhedron
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::contactCentroid(p3d_vector3 centroid)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::contactCentroid(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return 0;
  }

  unsigned int i;

  centroid[0]= centroid[1]= centroid[2]= 0.0;

  for(i=0; i<contacts.size(); ++i)
  {
    centroid[0]+= contacts[i].position[0];
    centroid[1]+= contacts[i].position[1];
    centroid[2]+= contacts[i].position[2];
  }

  centroid[0]/= contacts.size();
  centroid[1]/= contacts.size();
  centroid[2]/= contacts.size();

  return GP_OK;
}


//! Computes the stability score of the grasp.
//! If the grasp is unstable, the score is < 0.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::computeStability()
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::computeStability(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(contacts.size()<3)
  {
    stability= 0.0;
    return GP_OK;
  }

  unsigned int i;
  double (*_contacts)[3], (*_normals)[3], *_mu;

  _contacts= (double (*)[3]) new double[3*contacts.size()];
  _normals = (double (*)[3]) new double[3*contacts.size()];
  _mu     = (double *) new double[contacts.size()];

  for(i=0; i<contacts.size(); i++)
  {
     _contacts[i][0]= contacts[i].position[0];
     _contacts[i][1]= contacts[i].position[1];
     _contacts[i][2]= contacts[i].position[2];

     _normals[i][0]= contacts[i].normal[0];
     _normals[i][1]= contacts[i].normal[1];
     _normals[i][2]= contacts[i].normal[2];
     _mu[i]= contacts[i].mu;
  }

  stability= gpForce_closure_3D_grasp(_contacts, _normals, _mu, contacts.size(), (unsigned int) 6);

  delete [] _contacts;
  delete [] _normals;
  delete [] _mu;

  return GP_OK;
}

//! Removes the contact that are too close to an sharp edge.
//! For each contact of the grasp, the function considers the edges of the face the contact is on.
//! If one of the edge is not too "flat", the distance from the contact to the edge is computed and 
//! if the distance is above a threshold, the contact is regarded as too close to the edge.
//! \param angleThreshold the edge angle (in radians) above which an edge is regarded as not flat
//! \param distancethreshold the distance threshold
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::removeContactsTooCloseToEdge(double angleThreshold, double distancethreshold)
{
  bool remove;
  unsigned int i, j, i1, i2;
  double distance;
  p3d_vector3 p1, p2, closestPoint;
  p3d_polyhedre *poly= NULL;
  int face_edges[3];
  float angle;
  std::vector<gpContact> newContacts;


  for(i=0; i<contacts.size(); ++i)
  {
    remove= false;
    poly= contacts[i].surface;
    if(poly==NULL)
    {
      printf("%s: %d: gpGrasp::removeContactsTooCloseToEdge(): a contact of the grasp has a NULL surface\n",__FILE__,__LINE__);
      continue;
    }
    // compute the edges of the poly:
    if(poly->areEdgesAndNeighboursUpToDate==FALSE)
    {
      p3d_compute_edges_and_face_neighbours(poly);
    }

    //get the indices of the face edges:
    face_edges[0]= poly->the_faces[contacts[i].face].edges[0];
    face_edges[1]= poly->the_faces[contacts[i].face].edges[1];
    face_edges[2]= poly->the_faces[contacts[i].face].edges[2];

    //for each edge:
    for(j=0; j<3; ++j)
    {
      if(face_edges[j]==-1)
      {  continue;  }
      angle= poly->the_edges[face_edges[j]].angle;
      if(isnan(angle))
      {  continue;  }

      // if the edge is flat, skip the test
      if( fabs(angle) < angleThreshold )
      {  continue; }

      i1= poly->the_edges[face_edges[j]].point1 -1;
      i2= poly->the_edges[face_edges[j]].point2 -1;
      p3d_vectCopy(poly->the_points[i1], p1);
      p3d_vectCopy(poly->the_points[i2], p2);

      distance= gpPoint_to_line_segment_distance(contacts[i].position, p1, p2, closestPoint);
      if(distance < distancethreshold) 
      {  remove= true; }
    }
    if(remove==false)
    {  newContacts.push_back(contacts[i]);  }
  }

  contacts= newContacts;

  return GP_OK;
}

//! Computes the quality of the grasp.
//! For now, the quality is a weighted sum of a "force closure quality criterion"
//! and a score telling how close are the contact normals to the main grasping direction of the hand or gripper
//! (i.e. the (or main) direction along which the hand can exert a force). This last point is still
//! much too coarse. If the grasp does not verify force-closure, its global quality will be kept null.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::computeQuality()
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::computeQuality(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i;
  p3d_vector3 graspingDirection; //direction of the grasping force of the hand
  p3d_vector3 centroid;
//   p3d_vector3 closest;
  double (*_contacts)[3];
  double (*_normals)[3];
  double *_mu;


//   double edge_angle, dist_to_edge;
  //double weight1, weight2, weight3, weight4;
  double score2, score3;
  double minCurvatureScore;
  double fcWeight, directionWeight, curvatureWeight, configWeight, centroidWeight;
  double fcScore, directionScore, curvatureScore, configScore, centroidScore;
  p3d_polyhedre *poly= NULL;


  _contacts= new double[contacts.size()][3];
  _normals=  new double[contacts.size()][3];
  _mu     =  new double[contacts.size()];


  switch(hand_type)
  {
    case GP_GRIPPER: case GP_PR2_GRIPPER: case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      graspingDirection[0]= 1.0;
      graspingDirection[1]= 0.0;
      graspingDirection[2]= 0.0;
    break;
    default:
       printf("%s: %d: gpGrasp::computeQuality(): unimplemented or unknown hand type.\n", __FILE__, __LINE__);
       return GP_ERROR;
    break;
  }
 
   poly= object->o[0]->pol[0]->poly;
   
   //! check if the object volume properties were already computed:
   if(poly->volume==0)
   {   gpCompute_mass_properties(poly);  }
   else
   {
     if(poly->volume==0)
     {
       printf("%s: %d: gpGrasp::computeQuality(): the volume of \"%s\" should not be zero at this point.\n", __FILE__, __LINE__, poly->name);
       return GP_ERROR;
     }
   }

//   if(polyhedron==NULL)
//   {
//     printf("%s: %d: gpGrasp::computeQuality(): polyhedron is NULL.\n", __FILE__, __LINE__);
//     return 0;
//   }

  score2= 0.0;
  score3= 0.0;

  directionScore= 0.0; 
  curvatureScore= 0.0;
  for(i=0; i<contacts.size(); ++i)
  {
     _contacts[i][0]= contacts[i].position[0];
     _contacts[i][1]= contacts[i].position[1];
     _contacts[i][2]= contacts[i].position[2];

     _normals[i][0]= contacts[i].normal[0];
     _normals[i][1]= contacts[i].normal[1];
     _normals[i][2]= contacts[i].normal[2];
     _mu[i]= contacts[i].mu;

     directionScore+= fabs( graspingDirection[0]*_normals[i][0] + graspingDirection[1]*_normals[i][1] + graspingDirection[2]*_normals[i][2] );

     curvatureScore+= 1 - contacts[i].curvature;

     if( i==0 || (curvatureScore < minCurvatureScore) )
     {
        minCurvatureScore= curvatureScore;
     }
  }
  contactCentroid(centroid);
  centroidScore= p3d_vectDistance(poly->cmass, centroid);

  configScore= configCost();

  fcScore= stability; //gpForce_closure_3D_grasp(_contacts, _normals, _mu, contacts.size(), (unsigned int) 6);

  if(isnan(fcScore))                fcScore= 0.0;
  if(isnan(directionScore))  directionScore= 0.0;
  if(isnan(curvatureScore))  curvatureScore= 0.0;
  if(isnan(configScore))        configScore= 0.0;
  if(isnan(centroidScore))    centroidScore= 0.0;

  if(fcScore <= 0) 
  {   
    quality= 0.0;
    return GP_OK;
  }

  fcWeight= 1.0;
  directionWeight= 8.0;
  curvatureWeight= 2.0;
  configWeight= 1.0;
  centroidWeight= 1.0;

//   fcWeight= 01.0;
//   directionWeight= 8.0;
//   curvatureWeight= 0.0;
//   configWeight= 0.0;
//   centroidWeight= 0.0;

//   printf("fcScore= %f\n", fcScore);
//   printf("directionScore= %f\n", directionScore);
//   printf("curvatureScore= %f\n", curvatureScore);
//   printf("configScore= %f\n", configScore);
//   printf("centroidScore= %f\n", centroidScore);
  if(this->hand_type==GP_PR2_GRIPPER)
  {
    quality= minCurvatureScore;//stability + curvatureScore;
  }
  else
  {
    quality= fcWeight*fcScore + directionWeight*directionScore + curvatureWeight*curvatureScore + configWeight*configScore + centroidWeight*centroidScore;
    quality= curvatureWeight*minCurvatureScore + centroidWeight*1.0/centroidScore;
  }



  delete [] _contacts;
  delete [] _normals;
  delete [] _mu;

  return GP_OK;
}



//! Comparison function of the quality scores of two grasps.
//! \return true if quality of grasp 1 is less than quality of grasp 2
bool gpCompareGraspQuality(const gpGrasp &grasp1, const gpGrasp &grasp2)
{
  unsigned int i;
  double curvatureScore;
  std::list<double> curvatureScores1, curvatureScores2;

  for(i=0; i<grasp1.contacts.size(); ++i)
  {
    curvatureScore= 1.0 - grasp1.contacts.at(i).curvature;
    curvatureScores1.push_back(curvatureScore);
  }
  for(i=0; i<grasp2.contacts.size(); ++i)
  {
    curvatureScore= 1.0 - grasp2.contacts.at(i).curvature;
    curvatureScores2.push_back(curvatureScore);
  }

  curvatureScores1.sort();
  curvatureScores2.sort();

  while( curvatureScores1.size() > curvatureScores2.size() )
  {
    curvatureScores1.pop_front();
  }
  while( curvatureScores2.size() > curvatureScores1.size() )
  {
    curvatureScores2.pop_front();
  }

  return (curvatureScores1.front() < curvatureScores2.front()) ? true : false;
//   return (grasp1.quality < grasp2.quality) ? true : false;
}

//! Reversed comparison function of the quality scores of two grasps.
//! \return true if quality of grasp 1 is greater than quality of grasp 2
bool gpReversedCompareGraspQuality(const gpGrasp &grasp1, const gpGrasp &grasp2)
{
  return !(gpCompareGraspQuality(grasp1, grasp2));
}

//! Comparison function of the visibility scores of two grasps.
bool gpCompareGraspVisibility(const gpGrasp &grasp1, const gpGrasp &grasp2)
{
  return (grasp1.visibility < grasp2.visibility) ? true : false;
}

//! Comparison function of the number of contacts of two grasps.
bool gpCompareGraspNumberOfContacts(const gpGrasp &grasp1, const gpGrasp &grasp2)
{
  return (grasp1.contacts.size() < grasp2.contacts.size()) ? true : false;
}

//! Prints the content of a gpGrasp variable in the standard output.
int gpGrasp::print()
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::print(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i;

  printf("grasp: \n");
  printf("\t ID: %d (%p)\n", ID, this);
  printf("\t handID: %d \n", handID);
  printf("\t object: %s\n", object_name.c_str());
  printf("\t stability: %f \n", stability);
  printf("\t IKscore: %f \n", quality);
  printf("\t quality: %f \n", quality);
  printf("\t frame: [ %f %f %f %f \n", frame[0][0], frame[0][1], frame[0][2], frame[0][3]);
  printf("\t          %f %f %f %f \n", frame[1][0], frame[1][1], frame[1][2], frame[1][3]);
  printf("\t          %f %f %f %f \n", frame[2][0], frame[2][1], frame[2][2], frame[2][3]);
  printf("\t          %f %f %f %f ] \n", frame[3][0], frame[3][1], frame[3][2], frame[3][3]);
  printf("\t nb_contacts: %d \n", contacts.size());
  printf("\t contacts:\n");

  for(i=0; i<contacts.size(); i++)
  {
    printf("\t\t contact %d:\n", i);
    printf("\t\t\t position: [%f %f %f]\n",contacts[i].position[0],contacts[i].position[1],contacts[i].position[2]);
    printf("\t\t\t normal:   [%f %f %f]\n",contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]);
    printf("\t\t\t barycentric coords:   [%f %f %f]\n",contacts[i].baryCoords[0],contacts[i].baryCoords[1],contacts[i].baryCoords[2]);
    printf("\t\t\t finger: %d\n", contacts[i].fingerID);
  }

  printf("\t nb_dofs: %d \n", config.size());
  printf("\t configuration:\n");

  for(i=0; i<config.size(); i++)
  {
    printf("\t\t %f\n", config[i]);
  }

  printf("\t open configuration:\n");

  for(i=0; i<openConfig.size(); i++)
  {
    printf("\t\t %f\n", openConfig[i]);
  }

  return GP_OK;
}


//! Prints the content of a gpGrasp variable in a file.
//! \param filename name of the file
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::printInFile(const char *filename)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::printInFile(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(filename==NULL)
  { 
    printf("%s: %d: gpGrasp::printInFile(): input is NULL.\n", __FILE__, __LINE__);
    return GP_ERROR;
  }

  FILE *file= NULL;
  file= fopen(filename,"a+");
  
  if(file==NULL)
  { 
    printf("%s: %d: gpGrasp::printInFile(): can not open file \"%s\".\n", __FILE__, __LINE__,filename);
    return GP_ERROR; 
  }

  unsigned int i;

  fprintf(file, "grasp: \n");
  fprintf(file, "\t ID: %d (%p) \n", ID, this);
  fprintf(file, "\t handID: %d\n", handID);
  fprintf(file, "\t object: %s\n", object_name.c_str());
  fprintf(file, "\t stability: %f \n", stability);
  fprintf(file, "\t IKscore: %f \n", IKscore);
  fprintf(file, "\t quality: %f \n", quality);
  fprintf(file, "\t frame: [ %f %f %f %f \n", frame[0][0], frame[0][1], frame[0][2], frame[0][3]);
  fprintf(file, "\t          %f %f %f %f \n", frame[1][0], frame[1][1], frame[1][2], frame[1][3]);
  fprintf(file, "\t          %f %f %f %f \n", frame[2][0], frame[2][1], frame[2][2], frame[2][3]);
  fprintf(file, "\t          %f %f %f %f ] \n",frame[3][0], frame[3][1], frame[3][2], frame[3][3]);

  fprintf(file, "\t nb_contacts: %d \n", contacts.size());
  fprintf(file, "\t contacts:\n");

  for(i=0; i<contacts.size(); i++)
  {
    fprintf(file, "\t\t contact %d:\n", i);
    fprintf(file, "\t\t\t position: [%f %f %f]\n",contacts[i].position[0],contacts[i].position[1],contacts[i].position[2]);
    fprintf(file, "\t\t\t normal: [%f %f %f]\n",contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]);
    fprintf(file, "\t\t\t barycentric coords: [%f %f %f]\n",contacts[i].baryCoords[0],contacts[i].baryCoords[1],contacts[i].baryCoords[2]);
    fprintf(file, "\t\t\t finger: %d\n", contacts[i].fingerID);
  }

  fprintf(file, "\t nb_dofs: %d \n", config.size());
  fprintf(file, "\t configuration:\n");

  for(i=0; i<config.size(); i++)
  {
    fprintf(file, "\t\t %f\n", config[i]);
  }

  fprintf(file, "\t open configuration:\n");

  for(i=0; i<openConfig.size(); i++)
  {
    fprintf(file, "\t\t %f\n", openConfig[i]);
  }

  fclose(file); 

  return GP_OK;
}


//! Gives the direction of the wrist associated to the gpGrasp.
//! \param direction a p3d_vector3 that will be filled with the wrist direction
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::direction(p3d_vector3 direction) const
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::direction(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  p3d_matrix4 hand_frame;
  gpHand_properties handProp;

  handProp.initialize(hand_type);

  p3d_mat4Mult((p3d_matrix_type(*)[4]) frame, handProp.Tgrasp_frame_hand, hand_frame);

  //the direction of the hand is the Z axis:
  direction[0]= hand_frame[0][2];
  direction[1]= hand_frame[1][2];
  direction[2]= hand_frame[2][2];

  return GP_OK;
}



gpHand_properties::gpHand_properties()
{
  type= GP_HAND_NONE;
  nb_fingers= 0;
  nb_dofs= 0;
}


//! Initializes the geometric info for the selected hand type.
//! NB:
//! The convention for the wrist frame of the SAHands is (view from top, with direct frames):
//!     Z                                     Z
//!     ^                                     ^
//!     |                                     |
//!     |                                     |
//! || || ||                              || || ||
//! || || ||  / /                    \ \  || || || 
//! ||_||_|| / /                      \ \ ||_||_|| 
//! |         /   ---> Y       Y <---  \         |
//! |__LEFT__/                          |__RIGHT_|
//!
int gpHand_properties::initialize(gpHand_type hand_type)
{
  if(this==NULL)
  {
    printf("%s: %d: gpHand_properties::initialize(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  int i;
  p3d_vector3 t, axis;
  p3d_matrix4 T, Trinit, Tt1, Tt2, Tr1, Tr2, Tr3, Tr4, Tint1, Tint2;

  type= hand_type;

  switch(type)
  {
    case GP_GRIPPER:
       nb_fingers= 3;
       nb_dofs= 1;
       fingertip_distance =   0.04;
       fingertip_radius   =   0.0042;
       min_opening        =   0.01005;
       max_opening        =  0.075007;
       min_opening_jnt_value =   0.0;
       max_opening_jnt_value =   0.0325;

       qmin.resize(1);
       qmax.resize(1);
       qrest.resize(1);
       qopen.resize(1);
       qmin[0]= 0.0;
       qmax[0]= 0.0325;
       qrest[0]= qmax[0];
       qopen[0]= qmax[0];

       p3d_mat4Copy(p3d_mat4IDENTITY, Tgrasp_frame_hand);
       Tgrasp_frame_hand[2][3]= 0.007;

      //transformation grasp frame -> arm's wrist frame:
      /*
        x= z
        y= -x
        z= -y
      */
       T[0][0]=  0.0;  T[0][1]= -1.0;  T[0][2]=  0.0;  T[0][3]=  0.0;
       T[1][0]=  0.0;  T[1][1]=  0.0;  T[1][2]= -1.0;  T[1][3]=  0.0;
       T[2][0]=  1.0;  T[2][1]=  0.0;  T[2][2]=  0.0;  T[2][3]= -0.007;
       T[3][0]=  0.0;  T[3][1]=  0.0;  T[3][2]=  0.0;  T[3][3]=  1.0;

       nb_positions= 150;
       nb_directions= 24;
       nb_rotations= 10;
       max_nb_grasp_frames= 160000;
       edgeAngleThreshold= 80*DEGTORAD;
       edgeDistanceThreshold= 0.015;
    break;
    case GP_PR2_GRIPPER:
       nb_fingers= 2;
       nb_dofs= 1;
       fingertip_radius= 0.01;
       joint_fingertip_distance= 0.1;
       min_opening_jnt_value =   0.0;
       max_opening_jnt_value =   31.3981*DEGTORAD;

       p3d_mat4Copy(p3d_mat4IDENTITY, Tgrasp_frame_hand);
       Tgrasp_frame_hand[2][3]= 0.0;

       qmin.resize(1);
       qmax.resize(1);
       qrest.resize(1);
       qopen.resize(1);
       qmin[0]= 0.0;
       qmax[0]= max_opening_jnt_value;
       qrest[0]= qmax[0];
       qopen[0]= qmax[0];

       nb_positions= 1000;
       nb_directions= 6;
       nb_rotations= 8;
       max_nb_grasp_frames= 5000;
       edgeAngleThreshold= 80*DEGTORAD;
       edgeDistanceThreshold= 0.01; // be very careful with this value for pr2: As the jaws are not parallel they can not be very far from the object's border
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
       nb_fingers= 4;
       nb_dofs= 13;
       length_thumbBase =   0.0855;
       length_proxPha   =   0.067816;
       length_midPha    =   0.029980;
       length_distPha   =   0.029;
       fingertip_radius =   0.013;

       //compute the frames at the base of each finger:
       axis[0]=   0;
       axis[1]=   1;
       axis[2]=   0;

       if(type==GP_SAHAND_RIGHT)
       {
          p3d_mat4Copy(p3d_mat4IDENTITY, T);
          p3d_mat4Rot(T, axis, 23*DEGTORAD);
          T[0][3]= 0.08;
          T[1][3]= 0.02;
          T[2][3]= 0.23;
          p3d_matInvertXform(T, Tgrasp_frame_hand);
       }

       if(type==GP_SAHAND_LEFT)
       {
          p3d_mat4Copy(p3d_mat4IDENTITY, T);
          p3d_mat4Rot(T, axis, -23*DEGTORAD);
          T[0][3]= -0.08;
          T[1][3]= 0.02;
          T[2][3]= 0.23;
    
          T[0][0]= -T[0][0];
          T[1][0]= -T[1][0];
          T[2][0]= -T[2][0];

          T[0][1]= -T[0][1];
          T[1][1]= -T[1][1];
          T[2][1]= -T[2][1];
          p3d_matInvertXform(T, Tgrasp_frame_hand);
       }

       //initial axis swap (x'= -y, y'= z, z'= -x)
       axis[0]=  1;
       axis[1]= -1;
       axis[2]= -1;
       p3d_mat4Rot(Trinit, axis, 120*DEGTORAD);

       ////////////////////////////thumb///////////////////////////////////
       t[0]=  -0.0271;
       t[1]=        0;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);

       axis[0]=  0;
       axis[1]= -1;
       axis[2]=  0;
       p3d_mat4Rot(Tr1, axis, 90*DEGTORAD);

       t[0]=  -0.1126 - t[0];
       t[1]=  0.10282 - t[1];
       t[2]=    0.003 - t[2];
       p3d_mat4Trans(Tt2, t);

       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr2, axis, 35.2*DEGTORAD);

       axis[0]= 1;
       axis[1]= 0;
       axis[2]= 0;
       p3d_mat4Rot(Tr3, axis, -0.7*DEGTORAD);

       axis[0]= 0;
       axis[1]= 1;
       axis[2]= 0;
       p3d_mat4Rot(Tr4, axis, 270*DEGTORAD);

       p3d_matMultXform(Trinit, Tt1, Tint1);
       p3d_matMultXform(Tint1, Tr1, Tint2);
       p3d_matMultXform(Tint2, Tt2, Tint1);
       p3d_matMultXform(Tint1, Tr2, Tint2);
       p3d_matMultXform(Tint2, Tr3, Tint1);
       p3d_matMultXform(Tint1, Tr4, Twrist_finger[0]);

       ///////////////////////////forefinger///////////////////////////////////
       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr1, axis, 2.2*DEGTORAD);
       t[0]= -0.04025;
       t[1]=  0.15584;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);

       p3d_matMultXform(Trinit, Tt1, Tint1);
       p3d_matMultXform(Tint1, Tr1, Twrist_finger[1]);

       ///////////////////////////middle finger///////////////////////////////////
       t[0]=        0;
       t[1]=  0.16056;
       t[2]=    0.003;
       p3d_mat4Trans(Tt1, t);
       p3d_matMultXform(Trinit, Tt1, Twrist_finger[2]);

       ///////////////////////////ring finger///////////////////////////////////
       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(Tr1, axis, -1.76*DEGTORAD);
       t[0]= 0.04025;
       t[1]= 0.15584;
       t[2]=   0.003;
       p3d_mat4Trans(Tt1, t);

       p3d_mat4Mult(Trinit, Tt1, Tint1);
       p3d_mat4Mult(Tint1, Tr1, Twrist_finger[3]);


       if(type==GP_SAHAND_LEFT)
       {
         // mirror the frames wrt plane Oyz (NB: the frames are no more direct but 
         // this is not an issue and it allows to use exactly the same kinematic model for the left
         // and right fingers):
         for(i=0; i<4; i++)
         {
            Twrist_finger[i][0][0]=  -Twrist_finger[i][0][0];
            Twrist_finger[i][0][1]=  -Twrist_finger[i][0][1];
            Twrist_finger[i][0][2]=  -Twrist_finger[i][0][2];
            Twrist_finger[i][0][3]=  -Twrist_finger[i][0][3];
         }
       }

       // joint bounds
       qmin.resize(13);
       qmax.resize(13);
       //thumb:
       qmin[0]=   0.0*DEGTORAD;   qmax[0]= 90.0*DEGTORAD;
       qmin[1]= -15.0*DEGTORAD;   qmax[1]= 15.0*DEGTORAD;
       qmin[2]=  -4.0*DEGTORAD;   qmax[2]= 75.0*DEGTORAD;
       qmin[3]=   4.0*DEGTORAD;   qmax[3]= 75.0*DEGTORAD;
       //forefinger:
       qmin[4]= qmin[1];   qmax[4]= qmax[1];
       qmin[5]= qmin[2];   qmax[5]= qmax[2];
       qmin[6]= qmin[3];   qmax[6]= qmax[3];
       //middle finger:
       qmin[7]= qmin[1];   qmax[7]= qmax[1];
       qmin[8]= qmin[2];   qmax[8]= qmax[2];
       qmin[9]= qmin[3];   qmax[9]= qmax[3];
       //ring finger:
       qmin[10]= qmin[1];   qmax[10]= qmax[1];
       qmin[11]= qmin[2];   qmax[11]= qmax[2];
       qmin[12]= qmin[3];   qmax[12]= qmax[3];

       // rest configuration
       qrest.resize(13);
       qrest[0] = 90.0*DEGTORAD;
       qrest[1] = -0.3*DEGTORAD;
       qrest[2] = 22.63*DEGTORAD;
       qrest[3] = 17.56*DEGTORAD;

       qrest[4] = 11.20*DEGTORAD;
       qrest[5] = 26.92*DEGTORAD;
       qrest[6] = 33.49*DEGTORAD;

       qrest[7] = -7.27*DEGTORAD;
       qrest[8] = 41.39*DEGTORAD;
       qrest[9] = 33.93*DEGTORAD;

       qrest[10] = -3.14*DEGTORAD;
       qrest[11] = 38.17*DEGTORAD;
       qrest[12] = 49.86*DEGTORAD;

       // "open" configuration
       qopen.resize(13);
       qopen[0] = 90.0*DEGTORAD;
       qopen[1] = 0.0*DEGTORAD;
       qopen[2] = 0.0*DEGTORAD;
       qopen[3] = 10.0*DEGTORAD;

       qopen[4] = 0.0*DEGTORAD;
       qopen[5] = 0.0*DEGTORAD;
       qopen[6] = 10.0*DEGTORAD;

       qopen[7] = 0.0*DEGTORAD;
       qopen[8] = 0.0*DEGTORAD;
       qopen[9] = 10.0*DEGTORAD;

       qopen[10] = 0.0*DEGTORAD;
       qopen[11] = 0.0*DEGTORAD;
       qopen[12] = 10.0*DEGTORAD;

//        if(type==GP_SAHAND_RIGHT)
//        {
//           Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
//           Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]=  1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
//           Thand_wrist[2][0]=  1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
//           Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
//        }
// 
//        if(type==GP_SAHAND_LEFT)
//        {
//           Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
//           Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]= -1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
//           Thand_wrist[2][0]= -1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
//           Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
//        }

       //workspace (computed with gpSAHfinger_workspace_approximation (gpWorkspace.h));
       // DO NOT delete the commented lines:
       workspace.resize(51);
       workspace.at(0).setCenter(0.001412, 0.080912, -0.067195); 
       workspace.at(0).radius= 0.019585; 
       workspace.at(1).setCenter(-0.001021, 0.058473, -0.093971); 
       workspace.at(1).radius= 0.014200; 
       workspace.at(2).setCenter(0.000859, 0.049199, -0.066658); 
       workspace.at(2).radius= 0.011994; 
       workspace.at(3).setCenter(-0.012613, 0.102723, -0.051203); 
       workspace.at(3).radius= 0.010880; 
       workspace.at(4).setCenter(0.016190, 0.102220, -0.051203); 
       workspace.at(4).radius= 0.010833; 
       workspace.at(5).setCenter(0.000679, 0.038873, -0.108726); 
       workspace.at(5).radius= 0.009476; 
       workspace.at(6).setCenter(0.000652, 0.037382, -0.084485); 
       workspace.at(6).radius= 0.008987; 
       workspace.at(7).setCenter(-0.007173, 0.058422, -0.051025); 
       workspace.at(7).radius= 0.007686; 
       workspace.at(8).setCenter(0.001954, 0.111959, -0.041658); 
       workspace.at(8).radius= 0.007159; 
       workspace.at(9).setCenter(0.000504, 0.028863, -0.069647); 
       workspace.at(9).radius= 0.007036; 
       workspace.at(10).setCenter(0.011912, 0.075208, -0.092064); 
       workspace.at(10).radius= 0.007035; 
       workspace.at(11).setCenter(0.007173, 0.058422, -0.051025); 
       workspace.at(11).radius= 0.006661; 
       workspace.at(12).setCenter(-0.012407, 0.078335, -0.089351); 
       workspace.at(12).radius= 0.006654; 
       workspace.at(13).setCenter(-0.000487, 0.027918, -0.097234); 
       workspace.at(13).radius= 0.006443; 
       workspace.at(14).setCenter(0.000441, -0.025256, -0.073688); 
       workspace.at(14).radius= 0.006157; 
       workspace.at(15).setCenter(-0.018808, 0.096761, -0.067510); 
       workspace.at(15).radius= 0.006108; 
       workspace.at(16).setCenter(-0.021752, 0.111907, -0.039424); 
       workspace.at(16).radius= 0.005999; 
       workspace.at(17).setCenter(-0.000439, 0.025132, -0.115526); 
       workspace.at(17).radius= 0.005896; 
       workspace.at(18).setCenter(0.017734, 0.111966, -0.037724); 
       workspace.at(18).radius= 0.005756; 
       workspace.at(19).setCenter(-0.000409, -0.023406, -0.086968); 
       workspace.at(19).radius= 0.005706; 
       workspace.at(20).setCenter(0.001840, 0.105408, -0.058621); 
       workspace.at(20).radius= 0.005585; 
       workspace.at(21).setCenter(0.000391, 0.022376, -0.082304); 
       workspace.at(21).radius= 0.005455; 
       workspace.at(22).setCenter(-0.010050, 0.114869, -0.035422); 
       workspace.at(22).radius= 0.005386; 
       workspace.at(23).setCenter(-0.016721, 0.086020, -0.050548); 
       workspace.at(23).radius= 0.005339; 
       workspace.at(24).setCenter(0.019308, 0.099333, -0.067580); 
       workspace.at(24).radius= 0.005330; 
       workspace.at(25).setCenter(-0.017326, 0.089134, -0.081005); 
       workspace.at(25).radius= 0.005101; 
       workspace.at(26).setCenter(-0.009305, 0.058748, -0.076481); 
       workspace.at(26).radius= 0.005064; 
       workspace.at(27).setCenter(-0.001373, 0.078633, -0.092854); 
       workspace.at(27).radius= 0.004927; 
       workspace.at(28).setCenter(0.001672, 0.095803, -0.047830); 
       workspace.at(28).radius= 0.004844; 
       workspace.at(29).setCenter(-0.013546, 0.069686, -0.051490); 
       workspace.at(29).radius= 0.004837; 
       workspace.at(30).setCenter(-0.000332, -0.019016, -0.097213); 
       workspace.at(30).radius= 0.004636; 
       workspace.at(31).setCenter(-0.007952, 0.050207, -0.109392); 
       workspace.at(31).radius= 0.004620; 
       workspace.at(32).setCenter(0.000328, 0.018797, -0.103492); 
       workspace.at(32).radius= 0.004582; 
       workspace.at(33).setCenter(-0.012980, 0.066777, -0.080227); 
       workspace.at(33).radius= 0.004431; 
       workspace.at(34).setCenter(0.008423, 0.053182, -0.109122); 
       workspace.at(34).radius= 0.004421; 
       workspace.at(35).setCenter(0.012432, 0.063956, -0.080043); 
       workspace.at(35).radius= 0.004373; 
       workspace.at(36).setCenter(0.000311, 0.017819, -0.092971); 
       workspace.at(36).radius= 0.004344; 
       workspace.at(37).setCenter(0.010208, 0.116677, -0.033002); 
       workspace.at(37).radius= 0.004301; 
       workspace.at(38).setCenter(-0.011706, 0.060221, -0.065939); 
       workspace.at(38).radius= 0.004282; 
       workspace.at(39).setCenter(-0.000308, 0.017669, -0.070703); 
       workspace.at(39).radius= 0.004237; 
       workspace.at(40).setCenter(0.026347, 0.114121, -0.033002); 
       workspace.at(40).radius= 0.004088; 
       workspace.at(41).setCenter(0.016253, 0.083616, -0.085409); 
       workspace.at(41).radius= 0.004065; 
       workspace.at(42).setCenter(0.007594, 0.047947, -0.081269); 
       workspace.at(42).radius= 0.004057; 
       workspace.at(43).setCenter(-0.018584, 0.117336, -0.030475); 
       workspace.at(43).radius= 0.004032; 
       workspace.at(44).setCenter(-0.009026, 0.103173, -0.065694); 
       workspace.at(44).radius= 0.004032; 
       workspace.at(45).setCenter(-0.002073, 0.118781, -0.030475); 
       workspace.at(45).radius= 0.004032; 
       workspace.at(46).setCenter(-0.026724, 0.115754, -0.030475); 
       workspace.at(46).radius= 0.004032; 
       workspace.at(47).setCenter(0.009026, 0.103173, -0.065694); 
       workspace.at(47).radius= 0.003990; 
       workspace.at(48).setCenter(0.010833, 0.055731, -0.078510); 
       workspace.at(48).radius= 0.003963; 
       workspace.at(49).setCenter(0.012955, 0.066646, -0.102139); 
       workspace.at(49).radius= 0.003934; 
       workspace.at(50).setCenter(-0.005824, 0.036771, -0.096259); 
       workspace.at(50).radius= 0.003897; 

       nb_positions= 600;
       nb_directions= 16;
       nb_rotations=16;
       max_nb_grasp_frames= 300000;
       edgeAngleThreshold= 80*DEGTORAD;
       edgeDistanceThreshold= 0.025;
    break;
    default:
       printf("%s: %d: gpHand_properties::initalize(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       return GP_ERROR;
    break;
  }

  return GP_OK;
}


//! Initializes a gpHand_properties variable.
//! The function explores all the existing robots to find those with the specific
//! names defined in graspPlanning.h
//! The hand type is deduced from these names.
//! \return pointer to the hand robot, NULL otherwise
p3d_rob* gpHand_properties::initialize()
{
  if(this==NULL)
  {
    printf("%s: %d: gpHand_properties::initialize(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return NULL;
  }

  p3d_rob *hand_robot= NULL;

  hand_robot= p3d_get_robot_by_name((char*)GP_GRIPPER_ROBOT_NAME);
  if(hand_robot!=NULL)
  {
    type= GP_GRIPPER;
  }
  else
  {
    hand_robot= p3d_get_robot_by_name((char*)GP_PR2_GRIPPER_ROBOT_NAME);
    if(hand_robot!=NULL)
    {
       type= GP_PR2_GRIPPER;
    }
    else
    {
      hand_robot= p3d_get_robot_by_name((char*)GP_SAHAND_RIGHT_ROBOT_NAME);
      if(hand_robot!=NULL)
      {
        type= GP_SAHAND_RIGHT;
      }
      else
      {
        hand_robot= p3d_get_robot_by_name((char*)GP_SAHAND_LEFT_ROBOT_NAME);
        if(hand_robot!=NULL)
        {
        type= GP_SAHAND_LEFT;
        }
        else
        {
          printf("There must be a robot named \"%s\" or \"%s\" or \"%s\" or \"%s\".\n", GP_GRIPPER_ROBOT_NAME, GP_PR2_GRIPPER_ROBOT_NAME, GP_SAHAND_RIGHT_ROBOT_NAME, GP_SAHAND_LEFT_ROBOT_NAME);
          return NULL;
        }
      }
    }
  }


  initialize(type);

  return hand_robot;
}

//! Sets the matrix used to compute the arm wrist position from a given hand frame.
//! It depends on how the hand is linked to the arm and on the convention used for the arm's kinematics.
//! Default values are set when calling initialize() (for now they correspond to how the gripper or the SAHand are mounted on the PA-10).
//! \param T the new value of Thand_wrist
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpHand_properties::setThand_wrist(p3d_matrix4 T)
{
  if(this==NULL)
  {
    printf("%s: %d: gpHand_properties::setThand_wrist(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  p3d_mat4Copy(T, Thand_wrist);

  return GP_OK;
}

//! Sets the arm the hand is supposed to be mounted on. It is used to set Thand_wrist to one of some predefined values.
//! Default values are set when calling initialize() (for now they correspond to how the gripper or the SAHand are mounted on the PA-10).
//! \param arm_type the desired arm type
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpHand_properties::setArmType(gpArm_type arm_type)
{
  if(this==NULL)
  {
    printf("%s: %d: gpHand_properties::setArm(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
 
  p3d_vector3 axis;
  p3d_matrix4 R, T;

  p3d_mat4Copy(p3d_mat4IDENTITY, Thand_wrist);

  switch(arm_type)
  {
    case GP_PA10:
      switch(type)
      {
        case GP_GRIPPER:
          T[0][0]=  0.0;  T[0][1]= -1.0;  T[0][2]=  0.0;  T[0][3]=  0.0;
          T[1][0]=  0.0;  T[1][1]=  0.0;  T[1][2]= -1.0;  T[1][3]=  0.0;
          T[2][0]=  1.0;  T[2][1]=  0.0;  T[2][2]=  0.0;  T[2][3]=  -0.007;
          T[3][0]=  0.0;  T[3][1]=  0.0;  T[3][2]=  0.0;  T[3][3]=  1.0;

          axis[0]= 0;
          axis[1]= 0;
          axis[2]= 1;
          p3d_mat4Rot(R, axis, M_PI/8.0);
          p3d_matMultXform(R, T, Thand_wrist);
        break;
        case GP_SAHAND_RIGHT:
          Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
          Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]=  1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
          Thand_wrist[2][0]=  1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
          Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
        break;
        case GP_SAHAND_LEFT:
          Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
          Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]= -1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
          Thand_wrist[2][0]= -1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
          Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
        break;
        default:
          printf("%s: %d: gpHand_properties::setArm(): the hand type should have been set before.\n",__FILE__,__LINE__);
          return GP_ERROR;
        break;
      }
    break;
    case GP_LWR:
      switch(type)
      {
        case GP_GRIPPER:
          axis[0]= 0;
          axis[1]= 0;
          axis[2]= 1;
          p3d_mat4Rot(Thand_wrist, axis, 5*M_PI/8.0);
          Thand_wrist[0][3]= 0;
          Thand_wrist[1][3]= 0;
          Thand_wrist[2][3]= -0.268;
        break;
        case GP_SAHAND_RIGHT:
          printf("%s: %d: gpHand_properties::setArm(): transform is not yet defined for GP_LWR + GP_SAHAND_RIGHT.\n",__FILE__,__LINE__);
          return GP_ERROR;
        break;
        case GP_SAHAND_LEFT:
          printf("%s: %d: gpHand_properties::setArm(): transform is not yet defined for GP_LWR + GP_SAHAND_LEFT.\n",__FILE__,__LINE__);
          return GP_ERROR;
        break;
        default:
          printf("%s: %d: gpHand_properties::setArm(): the hand type should have been set before.\n",__FILE__,__LINE__);
          return GP_ERROR;
        break;
      }
    break;
    default:
      printf("%s: %d: gpHand_properties::setArm(): undefined or unimplemented arm type.\n",__FILE__,__LINE__);
      return GP_ERROR;
    break;
  }

  return GP_OK;
}

//! Draws different things that allow to visualize the dimension settings in a gpHand_properties class
//! by comparing the displayed things to the display of the Move3D model. Must be used in an OpenGL context.
//! \param pose the frame the things will be drawn in relatively to. Use the robot's wrist frame for instance.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpHand_properties::draw(p3d_matrix4 pose)
{
  if(this==NULL)
  {
    printf("%s: %d: gpHand_properties::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  static int ws= 0;
  unsigned int i;
  int result= GP_OK;
  float matGL[16];
  p3d_matrix4 Tgrasp_frame_hand_inv, T, T_inv;
  gpSAHandInfo data;

  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);

  p3d_matrix4_to_OpenGL_format(pose, matGL);
  glPushMatrix();
   glMultMatrixf(matGL);
    switch(type)
    {
      case GP_GRIPPER:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
        g3d_draw_frame(Tgrasp_frame_hand_inv, 0.1);

        g3d_set_color(Red, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_set_color(Green, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_set_color(Blue, NULL);
        g3d_draw_solid_sphere(-0.5*min_opening,0.0,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(-0.5*max_opening,0.0,  0.065, fingertip_radius, 20);
      break;
      case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
//         g3d_draw_frame(Tgrasp_frame_hand_inv, 0.1);

        p3d_mat4Mult(Tgrasp_frame_hand_inv, Thand_wrist, T);
        p3d_matInvertXform(T, T_inv);
//         g3d_draw_frame(T, 0.1);
//         g3d_draw_frame(T_inv, 0.1);
// //         p3d_matInvertXform(Thand_wrist, Thand_wrist_inv);
// //         g3d_draw_frame(Thand_wrist_inv, 0.1);

//         for(i=0; i<1; ++i)
        for(i=1; i<2; ++i)
        {
//           g3d_draw_frame(Twrist_finger[i], 0.05);
          p3d_to_gl_matrix(Twrist_finger[i], matGL);
          glPushMatrix();
            glMultMatrixf(matGL);
            switch(i)
            { 
              case 0:  g3d_set_color(Red, NULL);  break;
              case 1:  g3d_set_color(Green, NULL);  break;
              case 2:  g3d_set_color(Blue, NULL);  break;
              case 3:  g3d_set_color(Yellow, NULL);  break;
            }
//             gpDraw_SAHfinger_outer_workspace(data, 2*DEGTORAD);
//             for(unsigned int j=0; j<workspace.size(); ++j)
//             {
//               g3d_draw_solid_sphere(workspace[j].center[0],workspace[j].center[1],workspace[j].center[2], workspace[j].radius, 25);
//             }
printf("draw ws= %d\n",ws);
if(ws<2)
 {glDisable(GL_LIGHTING);
// glEnable(GL_LIGHTING);
             gpDraw_SAHfinger_outer_workspace(data, 2*DEGTORAD);
ws++;
}
else
{glEnable(GL_LIGHTING);
            for(unsigned int j=0; j<workspace.size(); ++j)
            {
              g3d_draw_solid_sphere(workspace[j].center[0],workspace[j].center[1],workspace[j].center[2], workspace[j].radius, 25);
            }
ws= 0;
}

/*
            glPushMatrix();
              glRotatef(-90, 1.0, 0.0, 0.0);
              g3d_set_color(Red, NULL);
              glTranslatef(0, 0, 0.5*length_proxPha);
              g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
              g3d_set_color(Green, NULL);
              glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
              g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
              g3d_set_color(Blue, NULL);
              glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
              g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
            glPopMatrix();*/
          glPopMatrix();
        }

      break;
      default:
       printf("%s: %d: gpHand_properties::draw(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       result= GP_ERROR;
      break;
    }

  glPopMatrix();
  glPopAttrib();


  return result;
}


//! Computes the "open" configuration of a grasp.
//! From the grasp configuration q, the hand is opened (in direction of the "qopen" configuration defined in gpHand_properties).
//! If there is a contact the opening is stopped and the DOF is set to 0.5*(qstart+qopen).
//! \param robot pointer to the robot hand (a freeflyer)
//! \param object pointer to the object (a freeflyer robot)
//! \param environment take into account the environment in the collision test or not
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpGrasp::computeOpenConfig(p3d_rob *robot, p3d_rob *object, bool environment)
{
  #ifdef GP_DEBUG
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::computeOpenConfig(): the calling instance is NULL.\n",__FILE__,__LINE__ );
    return GP_ERROR;
  }
  if(robot==NULL)
  {
    printf("%s: %d: gpGrasp::computeOpenConfig(): input robot is NULL.\n",__FILE__,__LINE__ );
    return GP_ERROR;
  }
  if(object==NULL)
  {
    printf("%s: %d: gpGrasp::computeOpenConfig(): input object is NULL.\n",__FILE__,__LINE__ );
    return GP_ERROR;
  }
  #endif

  unsigned int i, j,k, nbSteps;
  int result, nbChanges, col_test;
  double qnew[4]; // SAHand finger joint parameters to set (the first one is only needed by the thumb)
  p3d_matrix4 objectFrame;

  configPt config0, config;
  std::vector<bool> blocked, fingerBlocked;
  std::vector<double> q, qstart, qstop, delta;
  gpHand_properties handProp;

  //memorize the robot current configuration:
  config0= p3d_get_robot_config(robot);
  config= p3d_alloc_config(robot);

  nbSteps= 5;

  handProp.initialize(this->hand_type);

  p3d_get_freeflyer_pose(object, objectFrame);

  gpInverse_geometric_model_freeflying_hand(robot, objectFrame, this->frame, handProp, config);
  p3d_set_and_update_this_robot_conf(robot, config);

  this->openConfig= this->config;

  switch(this->hand_type)
  {
    case GP_GRIPPER:
      this->openConfig.at(0)= handProp.qopen.at(0);
    break;
    case GP_PR2_GRIPPER:
      if(this->config.size() !=1 || this->openConfig.size() !=1)
      {
        printf("%s: %d: gpGrasp::computeOpenConfig(): config vector has a bad size.\n",__FILE__,__LINE__ );
        break;
      }
      q.resize(1);
      qstart.resize(1);
      qstop.resize(1);
      delta.resize(1);
      qstart= this->config;
      qstop= handProp.qopen;
      for(i=0; i<delta.size(); ++i)
      {   delta[i]= ( qstop[i] - qstart[i] ) / ( ( double ) ( nbSteps ) );      }

      q= qstart;

      result= gpSet_hand_configuration(robot, handProp, q, true, 0);

      if(result==GP_ERROR)
      { printf("initial cfg is invalid\n"); break;  }

      for(j=1; j<=nbSteps; ++j)
      {
            q[0]+=  delta[0];
            result= gpSet_hand_configuration(robot, handProp, q, true, 0);

            if(result==GP_OK)
            {
              if(environment)
              { col_test= p3d_col_test_robot(robot, 0);  }
              else
              { col_test= p3d_col_test_robot_other(robot, object, 0) + p3d_col_test_self_collision(robot, 0); }
            }
            
            if( result==GP_ERROR || col_test )
            {
              if(result==GP_OK)
              {
                 q[0]= 0.5* ( qstart[0] + q[0] );
              }
              else
              {
                 q[0]-= delta[0];
              }  
              gpSet_hand_configuration(robot, handProp, q, true, 0);
            }
      }

      this->openConfig= q;
    break;
    case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      if(this->config.size() !=13 || this->openConfig.size() !=13)
      {
        printf("%s: %d: gpGrasp::computeOpenConfig(): config vector has a bad size.\n",__FILE__,__LINE__ );
        break;
      }
      q.resize(13);
      qstart.resize(13);
      qstop.resize(13);
      blocked.resize(13);
      delta.resize(13);
      fingerBlocked.resize(4);

      qstart= this->config;
      qstop= handProp.qopen;

      for(i=0; i<fingerBlocked.size(); ++i)
      {   fingerBlocked[i]= false;     }

      for(i=0; i<blocked.size(); ++i)
      {   blocked[i]= false;     }

      qstop[0]= qstart[0]; // for thumb
      blocked[0]= true;

      //block abduction joints:
//       for(i=0; i<4; ++i)
//       {  blocked[3*i+1]= true;   }

      for(i=0; i<delta.size(); ++i)
      {   delta[i]= ( qstop[i] - qstart[i] ) / ( ( double ) ( nbSteps ) );      }

      q= qstart;

      result= gpSet_hand_configuration(robot, handProp, q, true, 0);

      if(result==GP_ERROR)
      { printf("initial cfg is invalid\n"); break;  }

      for(j=1; j<=nbSteps; ++j)
      {
        nbChanges= 0;
        for(i=0; i<4; ++i) // for each finger
        {
          qnew[0]= qstart[0]; //for thumb only

          qnew[1]=  q[3*i+1];
          qnew[2]=  q[3*i+2];
          qnew[3]=  q[3*i+3];

          // for each DOF:
          for(k=1; k<4; ++k)
          {
            qnew[k]=  q[3*i+k] + delta[3*i+k];
            result= gpSet_SAHfinger_joint_angles(robot, handProp, qnew, i+1);

            if(result==GP_OK)
            {
              if(environment)
              { col_test= p3d_col_test_robot(robot, 0);  }
              else
              { col_test= p3d_col_test_robot_other(robot, object, 0) + p3d_col_test_self_collision(robot, 0); }
            }

            if( result==GP_ERROR || col_test )
            {
              qnew[k]= q[3*i+k];
              if(result==GP_OK) 
              {
                qnew[k]= 0.5* ( qstart[3*i+k] + q[3*i+k] );
              }
              gpSet_SAHfinger_joint_angles(robot, handProp, qnew, i+1);
            }
            else
            {
              nbChanges++;
             // keep the new value:
             q[3*i+k]= qnew[k];
            }
          }
        }
        if(nbChanges==0)
        {  break;  }
      }

      this->openConfig= q;
    break;
    default:
      printf("%s: %d: gpGrasp::computeOpenConfig(): this hand model is not defined.\n",__FILE__,__LINE__ );
      return GP_ERROR;
    break;
 }

 p3d_set_and_update_this_robot_conf(robot, config0);

 return GP_OK;
}

//! Default constructor of the class gpDoubleGrasp.
gpDoubleGrasp::gpDoubleGrasp()
{
  ID= 0;
  quality= 0.0;
}


//! Constructor of the class gpDoubleGrasp from two gpGrasp.
gpDoubleGrasp::gpDoubleGrasp(const gpGrasp &graspA, const gpGrasp &graspB)
{
  ID= 0;
  quality= 0.0;

  if(graspA.object!=graspB.object)
  {
    printf("%s: %d: gpDoubleGrasp::gpDoubleGrasp(): the two input grasps are not associated with the same object.\n",__FILE__,__LINE__);
    return;
  }

  grasp1= graspA;
  grasp2= graspB;
}



gpDoubleGrasp::gpDoubleGrasp(const gpDoubleGrasp &dgrasp)
{
  ID= dgrasp.ID;
  quality= dgrasp.quality;
  grasp1= dgrasp.grasp1;
  grasp2= dgrasp.grasp2;
}

gpDoubleGrasp::~gpDoubleGrasp()
{
}

  
//! Sets a double grasp from two gpGrasp.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDoubleGrasp::setFromSingleGrasps(const gpGrasp &graspA, const gpGrasp &graspB)
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::setFromSingleGrasps(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  ID= 0;
  quality= 0.0;

  if(graspA.object!=graspB.object)
  {
    printf("%s: %d: gpDoubleGrasp::setFromSingleGrasps(): the two input grasps are not associated with the same object.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  grasp1= graspA;
  grasp2= graspB;

  return GP_OK;
}


//! Copy operator of the class gpDoubleGrasp.
gpDoubleGrasp & gpDoubleGrasp::operator = (const gpDoubleGrasp &dgrasp)
{
  if( this!=&dgrasp )
  {
    ID= dgrasp.ID;
    quality= dgrasp.quality;
    grasp1= dgrasp.grasp1;
    grasp2= dgrasp.grasp2;
  }

  return *this;
}


//! Double grasp quality comparison operator.
bool gpDoubleGrasp::operator < (const gpDoubleGrasp &dgrasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::operator <: the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  return (quality < dgrasp.quality) ? true : false;
}


//! Draws all the contacts of a double grasp.
//! \param length lenght of each friction cone to draw
//! \param nb_slices number of segments of each cone discretization
//! \return GP_OK in case of success, GP_ERROR otherwise
//! The convention is (view from top, with direct frames)
 //!     Z                                     Z
 //!     ^                                     ^
 //!     |                                     |
 //!     |                                     |
 //! || || ||                              || || ||
 //! || || ||  / /                    \ \  || || || 
 //! ||_||_|| / /                      \ \ ||_||_|| 
 //! |         /   ---> Y       Y <---  \         |
 //! |__LEFT__/                          |__RIGHT_|
int gpDoubleGrasp::draw(double length, int nb_slices)
{  
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  

//   double normX, normY, normZ;
//   p3d_vector3 meanX, meanY, meanZ, X, Y, Z;
//   p3d_matrix4 gframeR, gframeL, T;
//   gpHand_properties handProp1, handProp2;

  grasp1.draw(0.1*length, nb_slices);
  grasp2.draw(0.1*length, nb_slices);

//   handProp1.initialize(grasp1.hand_type);
//   handProp2.initialize(grasp2.hand_type);
// 
//   if( ! ( (grasp1.hand_type==GP_SAHAND_RIGHT && grasp2.hand_type==GP_SAHAND_LEFT) ||
//         (grasp1.hand_type==GP_SAHAND_LEFT && grasp2.hand_type==GP_SAHAND_RIGHT) ) ) 
//   {
//     printf("%s: %d: gpDoubleGrasp::computeDirection(): the hand types should have been %s and %s.\n",__FILE__,__LINE__,gpHand_type_to_string(GP_SAHAND_RIGHT).c_str(),gpHand_type_to_string(GP_SAHAND_LEFT).c_str());
//     return GP_ERROR;
//   }
// 
//   if(grasp1.hand_type==GP_SAHAND_RIGHT)
//   {   
//     p3d_mat4Mult(grasp1.frame, handProp1.Tgrasp_frame_hand, gframeR); 
//     p3d_mat4Mult(grasp2.frame, handProp2.Tgrasp_frame_hand, gframeL); 
//   }
//   else
//   {   
//     p3d_mat4Mult(grasp1.frame, handProp1.Tgrasp_frame_hand, gframeL); 
//     p3d_mat4Mult(grasp2.frame, handProp2.Tgrasp_frame_hand, gframeR); 
//   }
// 
//   g3d_draw_frame(gframeR, 0.2);
//   g3d_draw_frame(gframeL, 0.2);

  return GP_OK;
}


//! Prints the content of a gpDoubleGrasp variable in the standard output.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDoubleGrasp::print()
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::print(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  printf("\t ID: %d (%p)\n", ID, this);
  printf("\t quality: %f \n", quality);
  printf("\t grasp1 \n");
  grasp1.print();
  printf("\n");
  printf("\t grasp2 \n");
  grasp2.print();

  return GP_OK;
}



//! Computes the "best" orientation to give to an object to grasp it from a given position with the double grasp
//!  and for a given pose of the robot torso. The function will try to align the best possible the hand 
//! frame to the torse pose.
//! \param torsoPose torso pose matrix in world coordinates
//! \param objectPose object pose matrix in world coordinates
//! Only the position part is used, the rotation part will be filled by the function.
//! \return GP_OK in case of success, GP_ERROR otherwise
//! NB: this function only works if the double grasp has one grasp for GP_SAHAND_RIGHT and 
//! one grasp for GP_SAHAND_LEFT.
//! The convention for the torso pose matrix is Z upward,, X points to the front of the torse and
//! Y to its left.
int gpDoubleGrasp::computeBestObjectOrientation(p3d_matrix4 torsoPose, p3d_matrix4 objectPose)
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::computeBestObjectOrientation(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  bool noSolution;
  unsigned int i;
  double normX, normY, normZ, angleX, angleY, angleZ, dot;
  p3d_vector3 X, Y, Z;
  p3d_vector3 handPositionR, handPositionL, XR, XL, YR, YL, ZR, ZL;
  p3d_vector3 objectPosition, torsoPosition, torsoDirection;
  p3d_matrix4 gframeR, gframeL, Tdgrasp, Tdgrasp_inv, newTorsoPose;
  gpHand_properties handProp1, handProp2;

  handProp1.initialize(grasp1.hand_type);
  handProp2.initialize(grasp2.hand_type);

  if( ! ( (grasp1.hand_type==GP_SAHAND_RIGHT && grasp2.hand_type==GP_SAHAND_LEFT) ||
        (grasp1.hand_type==GP_SAHAND_LEFT && grasp2.hand_type==GP_SAHAND_RIGHT) ||
        (grasp1.hand_type==GP_PR2_GRIPPER && grasp2.hand_type==GP_PR2_GRIPPER) ) )
  {
    printf("%s: %d: gpDoubleGrasp::computeBestObjectOrientation(): the hand types should have been %s and %s.\n",__FILE__,__LINE__,gpHand_type_to_string(GP_SAHAND_RIGHT).c_str(),gpHand_type_to_string(GP_SAHAND_LEFT).c_str());
    return GP_ERROR;
  }

  if(grasp1.hand_type==GP_SAHAND_RIGHT)
  {   
    p3d_mat4Mult(grasp1.frame, handProp1.Tgrasp_frame_hand, gframeR); 
    p3d_mat4Mult(grasp2.frame, handProp2.Tgrasp_frame_hand, gframeL); 
  }
  else
  {   
    p3d_mat4Mult(grasp1.frame, handProp1.Tgrasp_frame_hand, gframeL); 
    p3d_mat4Mult(grasp2.frame, handProp2.Tgrasp_frame_hand, gframeR); 
  }

  for(i=0; i<3; ++i)
  {
    X[i]= 0.5*(gframeR[i][0] - gframeL[i][0]);
    Y[i]= 0.5*(gframeR[i][1] - gframeL[i][1]);
    Z[i]= 0.5*(gframeR[i][2] + gframeL[i][2]);
  }

  p3d_mat4ExtractColumnX(gframeR, XR);
  p3d_mat4ExtractColumnX(gframeL, XL);
  p3d_mat4ExtractColumnY(gframeR, YR);
  p3d_mat4ExtractColumnY(gframeL, YL);
  p3d_mat4ExtractColumnZ(gframeR, ZR);
  p3d_mat4ExtractColumnZ(gframeL, ZL);
  p3d_mat4ExtractTrans(gframeR, handPositionR);
  p3d_mat4ExtractTrans(gframeL, handPositionL);

  dot= fabs(p3d_vectDotProd(XR, XL));
  angleX= fabs(acos(dot))*RADTODEG;

  dot= fabs(p3d_vectDotProd(YR, YL));
  angleY= fabs(acos(dot))*RADTODEG;

  dot= fabs(p3d_vectDotProd(ZR, ZL));
  angleZ= fabs(acos(dot))*RADTODEG;

  normX= p3d_vectNorm(X);
  normY= p3d_vectNorm(Y);
  normZ= p3d_vectNorm(Z);

//   printf("ID= %d\n",ID);
//   printf("norms %f %f %f \n",normX,normY,normZ);

  noSolution= false;

  p3d_vectSub(handPositionL, handPositionR, Y);
  normY= p3d_vectNorm(Y); 
  p3d_vectNormalize(Y, Y);
  

  if(angleZ > 10)
  { 
    for(i=0; i<3; ++i)
    {    Z[i]= 0.5*(ZR[i] + ZL[i]); }

    p3d_vectNormalize(Z, Z);
    p3d_vectXprod(Y, Z, X);

    p3d_vectNormalize(X, X);
    p3d_vectXprod(Z, X, Y);
  }
  else
  { 
    for(i=0; i<3; ++i)
    {   X[i]= 0.5*(XR[i] - XL[i]); }
    normX= p3d_vectNorm(X);
    if(normX > 1e-7)
    {
      p3d_vectNormalize(X, X);
    }
    else
    {
      p3d_vectCopy(XR, X);
    }
    p3d_vectXprod(X, Y, Z);
    p3d_vectXprod(Y, Z, X);
  }

  p3d_mat4Copy(p3d_mat4IDENTITY, Tdgrasp);
  if(noSolution)
  {
    for(i=0; i<3; ++i)
    {
      Tdgrasp[i][0]= gframeR[i][0];
      Tdgrasp[i][1]= gframeR[i][1];
      Tdgrasp[i][2]= gframeR[i][2];
    }
  }
  else
  {
    for(i=0; i<3; ++i)
    {
      Tdgrasp[i][0]= X[i];
      Tdgrasp[i][1]= Y[i];
      Tdgrasp[i][2]= Z[i];
    }
  }

//   p3d_mat4Print(Tdgrasp, "Tdgrasp");

  p3d_mat4ExtractTrans(torsoPose, torsoPosition);
  p3d_mat4ExtractTrans(objectPose, objectPosition);

  p3d_vectSub(objectPosition, torsoPosition, torsoDirection);

  if(p3d_vectNorm(torsoDirection) < 1e-7)
  {
    for(i=0; i<3; ++i)
    {
      torsoDirection[i]= torsoPose[i][0];
    }
  }

  torsoDirection[2]= 0.0;
  p3d_vectNormalize(torsoDirection, torsoDirection);

  X[0]= X[1]=0.0;
  X[2]= -1.0;
  p3d_vectCopy(torsoDirection, Z);

  p3d_vectXprod(Z, X, Y);
  p3d_vectNormalize(Y, Y);
 
  p3d_mat4Copy(p3d_mat4IDENTITY, newTorsoPose);
  for(i=0; i<3; ++i)
  {
    newTorsoPose[i][0]= X[i];
    newTorsoPose[i][1]= Y[i];
    newTorsoPose[i][2]= Z[i];
  }
//   p3d_mat4Print(newTorsoPose, "newTorsoPose");

  p3d_matInvertXform(Tdgrasp, Tdgrasp_inv);
//   p3d_mat4Print(Tdgrasp_inv, "Tdgrasp_inv");

  p3d_mat4Mult(newTorsoPose, Tdgrasp_inv, objectPose);

  for(i=0; i<3; ++i)
  {
    objectPose[i][3]= objectPosition[i];
  }

  return GP_OK;
}

//! Computes the stability score of a double grasp.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDoubleGrasp::computeStability()
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::computeStability(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  stability= MIN(grasp1.quality, grasp2.quality);

  return GP_OK;
}

//! Computes the quality score of a double grasp.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDoubleGrasp::computeQuality()
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::computeQuality(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  double IKscore;

  IKscore= MIN(grasp1.IKscore, grasp2.IKscore);

  quality= 0*distanceScore + 0.5*stability + 0.5*IKscore;

  return GP_OK;
}


//! Normalizes the distance score of the elements of a double grasp list and does distance= 1 - distance
//! in order to have a bigger distance score when the hands are the farthest from each other.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpNormalize_distance_score(std::list<gpDoubleGrasp> &list)
{
  if(list.size() < 2)
  {  return GP_OK;  }

  double d, dmin, dmax;
  std::list<gpDoubleGrasp>::iterator iter;

  dmin= dmax= list.front().distanceScore;

  for(iter=list.begin(); iter!=list.end(); iter++)
  {
     d= iter->distanceScore;
     if(d < dmin)
     {  dmin= d;  }
     if(d > dmax)
     {  dmax= d;  }
  }

  if( fabs(dmax-dmin) < 1e-9 )
  {
    for(iter=list.begin(); iter!=list.end(); iter++)
    {
      iter->distanceScore= 0.0;
    }
    return GP_OK;
  }


  for(iter=list.begin(); iter!=list.end(); iter++)
  {
     d= iter->distanceScore;
     iter->distanceScore= (d - dmin)/(dmax-dmin);
     iter->distanceScore= 1 - iter->distanceScore;
  }

  return GP_OK;
}


//! Normalizes the stability score of the elements of a double grasp list.
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpNormalize_stability(std::list<gpDoubleGrasp> &list)
{
  if(list.size() < 2)
  {  return GP_OK;  }

  double s, smin, smax;
  std::list<gpDoubleGrasp>::iterator iter;

  smin= smax= list.front().stability;

  for(iter=list.begin(); iter!=list.end(); iter++)
  {
     s= iter->stability;
     if(s < smin)
     {  smin= s;  }
     if(s > smax)
     {  smax= s;  }
  }

  if( fabs(smax-smin) < 1e-9 )
  {
    for(iter=list.begin(); iter!=list.end(); iter++)
    {
      iter->stability= 0.0;
    }
    return GP_OK;
  }


  for(iter=list.begin(); iter!=list.end(); iter++)
  {
     s= iter->stability;
     iter->stability= (s - smin)/(smax-smin);
  }

  return GP_OK;
}



