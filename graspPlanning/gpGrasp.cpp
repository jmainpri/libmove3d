
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <math.h>
#include <string>
#include <sstream>

//! Default constructor of the class gpContact.
gpContact::gpContact()
{
  ID= 0;
  surface= NULL;
  face= 0;   
  fingerID= 0;
  position[0]= position[1]= position[2]= 0.0;
  normal[0]= normal[1]= normal[2]= 0.0;
  baryCoords[0]= baryCoords[1]= baryCoords[2]= 0.0;
  curvature= 0;
  mu= 0.0;
}  


gpContact::gpContact(const gpContact &contact)
{
  ID        = contact.ID; 
  surface   = contact.surface; 
  face      = contact.face; 
  fingerID  = contact.fingerID;

  for(int i=0; i<3; i++)
  {
    position[i]  = contact.position[i];
    normal[i]    = contact.normal[i];
    baryCoords[i]= contact.baryCoords[i];
  }
  mu= contact.mu;
  curvature= contact.curvature;
}  

//! Copy operator of the class gpContact.
gpContact & gpContact::operator = (const gpContact &contact)
{
  if(this!=&contact)
  { 
    ID       = contact.ID; 
    surface  = contact.surface; 
    face     = contact.face;
    fingerID = contact.fingerID;

    for(int i=0; i<3; i++)
    {
      position[i]  = contact.position[i];
      normal[i]    = contact.normal[i];
      baryCoords[i]= contact.baryCoords[i];
    }
    mu= contact.mu;
    curvature= contact.curvature;
  }   

  return *this;
}


//! Draws a contact (position and friction cone).
//! \param length lenght of the friction cone to draw
//! \param nb_slices number of segments of the cone discretization
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpContact::draw(double length, int nb_slices)
{
//   #ifdef GP_DEBUG
//   if(surface==NULL)
//   {
//     printf("%s: %d: gpContact::draw((): no surface (p3d_polyhedre) is associated to the contact.\n", __FILE__, __LINE__);
//     return GP_ERROR;
//   }
//   #endif
  if(this==NULL)
  {
    printf("%s: %d: gpContact::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  p3d_matrix4 pose;
  GLfloat matGL[16];

  p3d_mat4Copy(p3d_mat4IDENTITY, pose);

  if(surface!=NULL)
  { 
    p3d_get_poly_pos(surface, pose );
  }

  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glPushAttrib(GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glPushMatrix();
//     glMultMatrixf(matGL);
    g3d_drawSphere(position[0], position[1], position[2], length/10.0);
    gpDraw_friction_cone(position, normal, mu, nb_slices, length);
  glPopMatrix();
  glPopAttrib();

  return GP_OK;
}

//! Computes the barycentric coordinates of the contact.
//! The barycentric coordinates are defined by the vertices of the triangle the contact belongs to.
//! If the triangle points are p1, p2 and p3 and the contact is p, we have:
//! p= a1*p1 + a2* p2 + a3*p3;
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpContact::computeBarycentricCoordinates()
{
  if(this==NULL)
  {
    printf("%s: %d: gpContact::computeBarycentricCoordinates(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(surface==NULL)
  {
    printf("%s: %d: gpContact::computeBarycentricCoordinates(): the p3d_polyhedre associated to the contact is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if( (face < 0) || (face > surface->nb_faces) )
  {
    printf("%s: %d: gpContact::computeBarycentricCoordinates(): the face index of the contact exceeds the face array dimension of the p3d_polyhedre.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  
  p3d_vector3 p, p1, p2, p3;
  p3d_matrix3 T, T_inv;

  p3d_vectCopy(surface->the_points[surface->the_faces[face].the_indexs_points[0]-1], p1);
  p3d_vectCopy(surface->the_points[surface->the_faces[face].the_indexs_points[1]-1], p2);
  p3d_vectCopy(surface->the_points[surface->the_faces[face].the_indexs_points[2]-1], p3);

  T[0][0]= p1[0];   T[0][1]= p2[0];    T[0][2]= p3[0]; 
  T[1][0]= p1[1];   T[1][1]= p2[1];    T[1][2]= p3[1]; 
  T[2][0]= p1[2];   T[2][1]= p2[2];    T[2][2]= p3[2]; 

  p3d_mat3Invert(T, T_inv);

  p3d_vec3Mat3Mult(T_inv, position, p);
  
  baryCoords[0]= p[0];
  baryCoords[1]= p[1];
  baryCoords[2]= p[2];

//   for(int i=0; i<3; i++)
//   {
//     p[i]= baryCoords[0]*p1[i] +  baryCoords[1]*p2[i] +  baryCoords[2]*p3[i];
//   }
// 
//   printf("[%f %f %f] [%f %f %f]\n",p[0],p[1],p[2],position[0],position[1],position[2]);

  return GP_OK;
}

//! Computes the curvature at the contact.
//! The computation is based on the barycentric coordinates of the contact and the curvature
//! at the mesh vertices (computed outside Move3D).
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpContact::computeCurvature()
{
  if(this==NULL)
  {
    printf("%s: %d: gpContact::computeCurvature(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  if(surface==NULL)
  {
    printf("%s: %d: gpContact::computeCurvature(): the p3d_polyhedre associated to the contact is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  double c1, c2, c3;

  c1= surface->curvatures[surface->the_faces[face].the_indexs_points[0]-1];
  c2= surface->curvatures[surface->the_faces[face].the_indexs_points[1]-1];
  c3= surface->curvatures[surface->the_faces[face].the_indexs_points[2]-1];

  curvature= baryCoords[0]*c1 + baryCoords[1]*c2 + baryCoords[2]*c3;

  return GP_OK;
}


//! Default constructor of the class gpGrasp
gpGrasp::gpGrasp()
{
  ID= 0;
  stability= 0;
  IKscore= 0;
  quality= 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, frame);
  handID= 0;
  object= NULL;
  body_index= 0;
  object_name= "none";
  hand_type= GP_HAND_NONE;
  collision_state= NOT_TESTED;
}

gpGrasp::gpGrasp(const gpGrasp &grasp)
{
  unsigned int i, j;

  ID= grasp.ID;
  stability= grasp.stability;
  IKscore= grasp.IKscore;
  quality= grasp.quality;
  handID= grasp.handID;

  for(i=0; i<4; i++)
  {      
    for(j=0; j<4; j++)
    {  frame[i][j]= grasp.frame[i][j];  }
  }

  object= grasp.object;
  body_index= grasp.body_index;
  object_name= grasp.object_name;
  hand_type= grasp.hand_type;
  collision_state= grasp.collision_state;

  contacts.resize(grasp.contacts.size());
  for(i=0; i<contacts.size(); i++)
  {  contacts[i]= grasp.contacts[i];  }

  config.resize(grasp.config.size());  
  for(i=0; i<config.size(); i++)
  {  config[i]= grasp.config[i];  }

  openConfig.resize(grasp.openConfig.size());  
  for(i=0; i<openConfig.size(); i++)
  {  openConfig[i]= grasp.openConfig[i];  }
}

gpGrasp::~gpGrasp()
{
  contacts.clear();
  config.clear();
}

//! Copy operator of the class gpGrasp.
gpGrasp & gpGrasp::operator = (const gpGrasp &grasp)
{
  unsigned int i, j;
  if( this!=&grasp )
  {
    ID= grasp.ID;
    stability= grasp.stability;
    IKscore= grasp.IKscore;
    quality= grasp.quality;
    handID= grasp.handID;

    for(i=0; i<4; i++)
    {
      for(j=0; j<4; j++)
      {  frame[i][j]= grasp.frame[i][j];  }
    }

    contacts.resize(grasp.contacts.size());
    for(i=0; i<contacts.size(); i++)
    {  contacts[i]= grasp.contacts[i];  }

    object= grasp.object;
    body_index= grasp.body_index;
    object_name= grasp.object_name;
    hand_type= grasp.hand_type;
    collision_state= grasp.collision_state;

    config.resize(grasp.config.size());  
    for(i=0; i<config.size(); i++)
    {  config[i]= grasp.config[i];   }

    openConfig.resize(grasp.openConfig.size());  
    for(i=0; i<openConfig.size(); i++)
    {  openConfig[i]= grasp.openConfig[i];   }
  }

  return *this;
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
  double q[4];
  p3d_vector3 p,fingerpad_normal;
  p3d_matrix4 pose, Twrist;
  GLfloat matGL[16];
  gpHand_properties hand;

  if(object!=NULL)
  {   p3d_get_body_pose(object, body_index, pose);  }
  else
  {   p3d_mat4Copy(p3d_mat4IDENTITY, pose);   }

  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glPushAttrib(GL_LIGHTING_BIT);
  glPushMatrix();
    glMultMatrixf(matGL);

    for(i=0; i<contacts.size(); ++i)
    {
      if(i==0) { glColor3f(1.0, 0.0, 0.0);  }
      if(i==1) { glColor3f(0.0, 1.0, 0.0);  }
      if(i==2) { glColor3f(0.0, 0.0, 1.0);  }
      if(i==3) glColor3f(1.0, 1.0, 0.0);
      contacts[i].draw(length, nb_slices);
    }

//     g3d_draw_frame(frame, 4*length);

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
    case GP_GRIPPER:
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
//! \param grasp
//! \return the computed distance 
double gpGrasp::distance(const gpGrasp &grasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::distance(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return 0;
  }

  return p3d_mat4Distance(frame, (p3d_matrix_type(*)[4]) grasp.frame, 1.0, 10.0);
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

  unsigned int i, j, v1, v2;
  p3d_vector3 graspingDirection; //direction of the grasping force of the hand
  p3d_vector3 centroid;
//   p3d_vector3 closest;
  double (*_contacts)[3], (*_normals)[3], *_mu;
//   double edge_angle, dist_to_edge;
  double weight1, weight2, weight3, weight4;
  double score1, score2, score3, score4;
  double fcWeight, directionWeight, curvatureWeight, configWeight, centroidWeight;
  double fcScore, directionScore, curvatureScore, configScore, centroidScore;
  p3d_face triangle;
  p3d_polyhedre *poly= NULL;

  _contacts= (double (*)[3]) new double[3*contacts.size()];
  _normals = (double (*)[3]) new double[3*contacts.size()];
  _mu     = (double *) new double[contacts.size()];

  switch(hand_type)
  {
    case GP_GRIPPER: case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
      graspingDirection[0]= 1.0;
      graspingDirection[1]= 0.0;
      graspingDirection[2]= 0.0;
    break;
    default:
       printf("%s: %d: gpGrasp::computeQuality(): unimplemented or unknown hand type.\n", __FILE__, __LINE__);
       return GP_ERROR;
    break;
  }
 
   poly= object->o[body_index]->pol[0]->poly;
   
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
  for(i=0; i<contacts.size(); i++)
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
  directionWeight= 1.0;
  curvatureWeight= 6.0;
  configWeight= 1.0;
  centroidWeight= 1.0;

//   printf("fcScore= %f\n", fcScore);
//   printf("directionScore= %f\n", directionScore);
//   printf("curvatureScore= %f\n", curvatureScore);
//   printf("configScore= %f\n", configScore);
//   printf("centroidScore= %f\n", centroidScore);

  quality= fcWeight*fcScore + directionWeight*directionScore + curvatureWeight*curvatureScore + configWeight*configScore + centroidWeight*centroidScore;

  delete [] _contacts;
  delete [] _normals;
  delete [] _mu;

  return GP_OK;
}


//! Grasp quality comparison operator.
bool gpGrasp::operator < (const gpGrasp &grasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::operator <: the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  return (quality < grasp.quality) ? true : false;
}

//! Grasp quality comparison operator.
bool gpGrasp::operator > (const gpGrasp &grasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::operator >: the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  return (quality > grasp.quality) ? true : false;
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
int gpGrasp::direction(p3d_vector3 direction)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::direction(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  p3d_matrix4 hand_frame;
  gpHand_properties handProp;

  handProp.initialize(hand_type);

  p3d_mat4Mult(frame, handProp.Tgrasp_frame_hand, hand_frame);

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
  p3d_matrix4 R, T, Trinit, Tt1, Tt2, Tr1, Tr2, Tr3, Tr4, Tint1, Tint2;

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
       qmin[0]= 0.0;
       qmax[0]= 0.0325;
       qrest[0]= qmax[0];

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
       T[2][0]=  1.0;  T[2][1]=  0.0;  T[2][2]=  0.0;  T[2][3]=  -0.007;
       T[3][0]=  0.0;  T[3][1]=  0.0;  T[3][2]=  0.0;  T[3][3]=  1.0;

//        T[0][0]=  0.0;  T[0][1]=  0.0;  T[0][2]=  1.0;  T[0][3]=  0.0;
//        T[1][0]= -1.0;  T[1][1]=  0.0;  T[1][2]=  0.0;  T[1][3]=  0.0;
//        T[2][0]=  0.0;  T[2][1]= -1.0;  T[2][2]=  0.0;  T[2][3]=  0.0;
//        T[3][0]=  0.0;  T[3][1]=  0.0;  T[3][2]=  0.0;  T[3][3]=  1.0;
//        Thand_wrist[0][0]=  0.0;  Thand_wrist[0][1]= -1.0;  Thand_wrist[0][2]=  0.0;  Thand_wrist[0][3]=  0.0; 
//        Thand_wrist[1][0]=  0.0;  Thand_wrist[1][1]=  0.0;  Thand_wrist[1][2]= -1.0;  Thand_wrist[1][3]=  0.0;
//        Thand_wrist[2][0]=  1.0;  Thand_wrist[2][1]=  0.0;  Thand_wrist[2][2]=  0.0;  Thand_wrist[2][3]=  0.065; 
//        Thand_wrist[3][0]=  0.0;  Thand_wrist[3][1]=  0.0;  Thand_wrist[3][2]=  0.0;  Thand_wrist[3][3]=  1.0;


       axis[0]= 0;
       axis[1]= 0;
       axis[2]= 1;
       p3d_mat4Rot(R, axis, M_PI/8.0);
       p3d_matMultXform(R, T, Thand_wrist);
       //p3d_mat4Copy(T, Thand_wrist);

       nb_positions= 2100;
       nb_directions= 12;
       nb_rotations= 6;
       max_nb_grasp_frames= 160000;

       nb_positions= 1000;
       nb_directions= 12;
       nb_rotations= 6;
       max_nb_grasp_frames= 10000;
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
       qmin[1]= -20.0*DEGTORAD;   qmax[1]= 20.0*DEGTORAD;
       qmin[2]= -19.0*DEGTORAD;   qmax[2]= 90.0*DEGTORAD;
       qmin[3]=   0.0*DEGTORAD;   qmax[3]= 70.0*DEGTORAD;
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

       if(type==GP_SAHAND_RIGHT)
       {
          Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
          Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]=  1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
          Thand_wrist[2][0]=  1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
          Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
       }

       if(type==GP_SAHAND_LEFT)
       {
          Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
          Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]= -1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
          Thand_wrist[2][0]= -1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.15;
          Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
       }

       //workspace (computed with gpSAHfinger_workspace_approximation (gpWorkspace.h));
       // DO NOT delete the commented lines:
       workspace.resize(15);
       workspace.at(0).setCenter(-0.000000, 0.082053, -0.055491); 
       workspace.at(0).radius= 0.027800; 
       workspace.at(1).setCenter(-0.000000, 0.054309, -0.092300); 
       workspace.at(1).radius= 0.018294; 
       workspace.at(2).setCenter(-0.000000, 0.040406, -0.051358); 
       workspace.at(2).radius= 0.013861; 
       workspace.at(3).setCenter(-0.022472, 0.105721, -0.030710); 
       workspace.at(3).radius= 0.013178; 
       workspace.at(4).setCenter(0.022472, 0.105721, -0.030710); 
       workspace.at(4).radius= 0.013178; 
       workspace.at(5).setCenter(0.000000, -0.037106, -0.076797); 
       workspace.at(5).radius= 0.012761; 
       workspace.at(6).setCenter(-0.000000, 0.113220, -0.022672); 
       workspace.at(6).radius= 0.011154; 
       workspace.at(7).setCenter(-0.000000, 0.031050, -0.074148); 
       workspace.at(7).radius= 0.010695; 
       workspace.at(8).setCenter(0.000000, -0.030234, -0.100747); 
       workspace.at(8).radius= 0.010446; 
       workspace.at(9).setCenter(-0.000000, 0.030260, -0.109865); 
       workspace.at(9).radius= 0.010436; 
       workspace.at(10).setCenter(0.000000, -0.029985, -0.052456); 
       workspace.at(10).radius= 0.010306; 
       workspace.at(11).setCenter(-0.007150, 0.050878, -0.031797); 
       workspace.at(11).radius= 0.009450; 
       workspace.at(12).setCenter(-0.016215, 0.076288, -0.088855); 
       workspace.at(12).radius= 0.008588; 
       workspace.at(13).setCenter(0.016215, 0.076288, -0.088855); 
       workspace.at(13).radius= 0.008588; 
       workspace.at(14).setCenter(0.009846, 0.055841, -0.032481); 
       workspace.at(14).radius= 0.008270; 
/*
       workspace.resize(25);

       workspace.at(0).setCenter(-0.003262, 0.062235, -0.082269); 
       workspace.at(0).radius= 0.017917; 
       workspace.at(1).setCenter(-0.000000, 0.037457, -0.100328); 
       workspace.at(1).radius= 0.012837; 
       workspace.at(2).setCenter(0.017101, 0.080452, -0.075329); 
       workspace.at(2).radius= 0.010165; 
       workspace.at(3).setCenter(-0.019116, 0.082799, -0.073510); 
       workspace.at(3).radius= 0.009327; 
       workspace.at(4).setCenter(-0.000000, 0.086247, -0.072016); 
       workspace.at(4).radius= 0.008119; 
       workspace.at(5).setCenter(-0.000000, 0.020569, -0.112085); 
       workspace.at(5).radius= 0.007058; 
       workspace.at(6).setCenter(-0.000000, 0.020580, -0.090066); 
       workspace.at(6).radius= 0.006915; 
       workspace.at(7).setCenter(0.012526, 0.054254, -0.100304); 
       workspace.at(7).radius= 0.006799; 
       workspace.at(8).setCenter(0.018700, 0.069791, -0.089111); 
       workspace.at(8).radius= 0.006296; 
       workspace.at(9).setCenter(0.007692, 0.039572, -0.083129); 
       workspace.at(9).radius= 0.006123; 
       workspace.at(10).setCenter(-0.012065, 0.052257, -0.102257); 
       workspace.at(10).radius= 0.006095; 
       workspace.at(11).setCenter(-0.007466, 0.038409, -0.082415); 
       workspace.at(11).radius= 0.005947; 
       workspace.at(12).setCenter(0.017423, 0.065023, -0.071193); 
       workspace.at(12).radius= 0.005712; 
       workspace.at(13).setCenter(0.011502, 0.093678, -0.067768); 
       workspace.at(13).radius= 0.005494; 
       workspace.at(14).setCenter(0.026015, 0.090725, -0.067768); 
       workspace.at(14).radius= 0.005397; 
       workspace.at(15).setCenter(0.000293, -0.016784, -0.089674); 
       workspace.at(15).radius= 0.005369; 
       workspace.at(16).setCenter(-0.011481, 0.093508, -0.066763); 
       workspace.at(16).radius= 0.005356; 
       workspace.at(17).setCenter(-0.000000, 0.015246, -0.101050); 
       workspace.at(17).radius= 0.005193; 
       workspace.at(18).setCenter(-0.027936, 0.091375, -0.066111); 
       workspace.at(18).radius= 0.005002; 
       workspace.at(19).setCenter(0.011778, 0.047237, -0.091005); 
       workspace.at(19).radius= 0.004875; 
       workspace.at(20).setCenter(-0.021546, 0.075141, -0.086911); 
       workspace.at(20).radius= 0.004830; 
       workspace.at(21).setCenter(0.014649, 0.054671, -0.070058); 
       workspace.at(21).radius= 0.004791; 
       workspace.at(22).setCenter(0.000957, 0.054813, -0.103240); 
       workspace.at(22).radius= 0.004726; 
       workspace.at(23).setCenter(-0.001691, 0.096858, -0.065418); 
       workspace.at(23).radius= 0.004489; 
       workspace.at(24).setCenter(0.002142, 0.030632, -0.084668); 
       workspace.at(24).radius= 0.004380; */
//        workspace.at(25).setCenter(0.012673, 0.047298, -0.075590); 
//        workspace.at(25).radius= 0.004272; 
//        workspace.at(26).setCenter(0.017049, 0.059458, -0.090737); 
//        workspace.at(26).radius= 0.004263; 
//        workspace.at(27).setCenter(0.000000, -0.012191, -0.098303); 
//        workspace.at(27).radius= 0.004234; 
//        workspace.at(28).setCenter(-0.008784, 0.083570, -0.082231); 
//        workspace.at(28).radius= 0.004122; 
//        workspace.at(29).setCenter(-0.017470, 0.060924, -0.099026); 
//        workspace.at(29).radius= 0.004093; 
//        workspace.at(30).setCenter(-0.006076, 0.028586, -0.085191); 
//        workspace.at(30).radius= 0.004077; 
       nb_positions= 500;
       nb_directions= 6;
       nb_rotations= 6;
       max_nb_grasp_frames= 5000;
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
        printf("There must be a robot named \"%s\" or \"%s\" or \"%s\".\n", GP_GRIPPER_ROBOT_NAME, GP_SAHAND_RIGHT_ROBOT_NAME, GP_SAHAND_LEFT_ROBOT_NAME);
        return NULL;
      }
    }
  }

  initialize(type);

  return hand_robot;
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
  static int ws= FALSE;

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
        g3d_draw_frame(Tgrasp_frame_hand_inv, 0.1);

        p3d_mat4Mult(Tgrasp_frame_hand_inv, Thand_wrist, T);
        p3d_matInvertXform(T, T_inv);
//         g3d_draw_frame(T, 0.1);
//         g3d_draw_frame(T_inv, 0.1);
// //         p3d_matInvertXform(Thand_wrist, Thand_wrist_inv);
// //         g3d_draw_frame(Thand_wrist_inv, 0.1);

//         for(i=0; i<1; ++i)
        for(i=0; i<2; ++i)
        {
          g3d_draw_frame(Twrist_finger[i], 0.05);
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
   gpDraw_SAHfinger_outer_workspace(data, 2*DEGTORAD);
            for(unsigned int j=0; j<workspace.size(); ++j)
            {
              g3d_draw_solid_sphere(workspace[j].center[0],workspace[j].center[1],workspace[j].center[2], workspace[j].radius, 25);
            }/*
printf("draw ws= %d\n",ws);
if(ws==TRUE)
 {//glDisable(GL_LIGHTING);
             gpDraw_SAHfinger_outer_workspace(data, 2*DEGTORAD);
ws= FALSE;
}
else
{glEnable(GL_LIGHTING);
            for(unsigned int j=0; j<workspace.size(); ++j)
            {
              g3d_draw_solid_sphere(workspace[j].center[0],workspace[j].center[1],workspace[j].center[2], workspace[j].radius, 25);
            }
ws= TRUE;
}*/

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

//! Double grasp quality comparison operator.
bool gpDoubleGrasp::operator > (const gpDoubleGrasp &dgrasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::operator >: the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  return (quality > dgrasp.quality) ? true : false;
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
        (grasp1.hand_type==GP_SAHAND_LEFT && grasp2.hand_type==GP_SAHAND_RIGHT) ) ) 
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



