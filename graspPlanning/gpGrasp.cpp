
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
    position[i]= contact.position[i];
    normal[i]  = contact.normal[i];
  }
  mu= contact.mu;
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
      position[i]= contact.position[i];
      normal[i]  = contact.normal[i];
    }
    mu= contact.mu;
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
    g3d_drawSphere(position[0], position[1], position[2] , length/10.0, Blue, NULL);
    gpDraw_friction_cone(position, normal, mu, nb_slices, length);
  glPopMatrix();
  glPopAttrib();

  return GP_OK;
}



//! Default constructor of the class gpGrasp
gpGrasp::gpGrasp()
{
  ID= 0;
  quality= 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, frame);
  handID= 0;
  polyhedron= NULL;
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
  quality= grasp.quality;
  handID= grasp.handID;

  for(i=0; i<4; i++)
  {      
    for(j=0; j<4; j++)
    {  frame[i][j]= grasp.frame[i][j];  }
  }

  polyhedron= grasp.polyhedron;
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

    polyhedron= grasp.polyhedron;
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

//   for(i=0; i<contacts.size();i++)
//   {
//     if(i==0) glColor3f(1.0, 0.0, 0.0);
//     if(i==1) glColor3f(0.0, 1.0, 0.0);
//     if(i==2) glColor3f(0.0, 0.0, 1.0);
//     if(i==3) glColor3f(1.0, 1.0, 0.0);
//     if(i==4) glColor3f(1.0, 0.0, 1.0);
//     contacts[i].draw(length, nb_slices);
//   }

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
      if(i==0) glColor3f(1.0, 0.0, 0.0);
      if(i==1) glColor3f(0.0, 1.0, 0.0);
      if(i==2) glColor3f(0.0, 0.0, 1.0);
      if(i==3) glColor3f(1.0, 1.0, 0.0);
      contacts[i].draw(length, nb_slices);
    }

    g3d_draw_frame(frame, 4*length);

    if(hand_type==GP_SAHAND_RIGHT)
    { 
      hand.initialize(GP_SAHAND_RIGHT);
      p3d_mat4Mult(frame, hand.Tgrasp_frame_hand, Twrist);
      q[0]= config.at(0);
      for(i=0; i<4; ++i)
      {
        q[1]= config.at(3*i+1);
        q[2]= config.at(3*i+2);
        q[3]= config.at(3*i+3);
        gpSAHfinger_forward_kinematics(Twrist, hand, q, p, fingerpad_normal, i+1);
        g3d_drawSphere(p[0], p[1], p[2], 0.008, Yellow, NULL);
        g3d_drawOneLine(p[0], p[1], p[2], p[0]+0.05*fingerpad_normal[0], p[1]+0.05*fingerpad_normal[1], p[2]+0.05*fingerpad_normal[2], Yellow, NULL);
//         for(unsigned int j=0; j<contacts.size(); ++j)
//         {
//           if( contacts[j].fingerID==(i+1) )
//           {
//             printf(" [%f %f %f]\n",contacts[j].normal[0], contacts[j].normal[1], contacts[j].normal[2]);
//             printf("dot= %f\n",p3d_vectDotProd(contacts[j].normal,fingerpad_normal));
//           }
//           
//         }
      }
    }

  glPopMatrix();
  glPopAttrib();
  
  return GP_OK;
}


//! Computes a cost for the given grasping configuration.
//! The biggest it is, the better it is.
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
       printf("%s: %d: gpGrasp::computeQuality(): unimplemented or unknown hand type.\n", __FILE__, __LINE__);
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

//! Computes and returns the quality --stability criterion-- of the grasp.
//! For now, the quality is a weighted sum of a "force closure quality criterion"
//! and a score telling how close are the contact normals to the main grasping direction of the hand or gripper
//! (i.e. the (or main) direction along which the hand can exert a force). This last point is still
//! much too coarse. If the grasp does not verify force-closure, its global quality will be kept null.
double gpGrasp::computeQuality()
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::computeQuality(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  unsigned int i, j, v1, v2;
  p3d_vector3 graspingDirection; //direction of the grasping force of the hand
  p3d_vector3 closest;
  double (*_contacts)[3], (*_normals)[3], *_mu;
  double edge_angle, dist_to_edge;
  double weight1, weight2, weight3, weight4;
  double score1, score2, score3, score4;
  p3d_face triangle;

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
       return 0;
    break;
  }
 
  if(polyhedron==NULL)
  {
    printf("%s: %d: gpGrasp::computeQuality(): polyhedron is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  score2= 0.0;
  score3= 0.0;
  for(i=0; i<contacts.size(); i++)
  {
     _contacts[i][0]= contacts[i].position[0];
     _contacts[i][1]= contacts[i].position[1];
     _contacts[i][2]= contacts[i].position[2];

     _normals[i][0]= contacts[i].normal[0];
     _normals[i][1]= contacts[i].normal[1];
     _normals[i][2]= contacts[i].normal[2];
     _mu[i]= contacts[i].mu;

     score2+= fabs( graspingDirection[0]*_normals[i][0] + graspingDirection[1]*_normals[i][1] + graspingDirection[2]*_normals[i][2] );

     triangle= polyhedron->the_faces[contacts[i].face];
     for(j=0; j<3; j++)
     {
       if(triangle.edges[j]==-1)
       {
         printf("%s: %d: gpGrasp::computeQuality(): the edges of \"%s\" were not properly computed.\n",__FILE__,__LINE__,polyhedron->name);
         continue;
       }

       v1= polyhedron->the_edges[triangle.edges[j]].point1 - 1;
       v2= polyhedron->the_edges[triangle.edges[j]].point2 - 1;

       if( (v1 > polyhedron->nb_points-1) || (v2 > polyhedron->nb_points-1) )
       if(triangle.edges[j]==-1)
       {
         printf("%s: %d: gpGrasp::computeQuality(): the edges of \"%s\" were not properly computed.\n",__FILE__,__LINE__,polyhedron->name);
         continue;
       }

       edge_angle= polyhedron->the_edges[triangle.edges[j]].angle;

       dist_to_edge= gpPoint_to_line_segment_distance(contacts[i].position, polyhedron->the_points[v1], polyhedron->the_points[v2], closest);
       score3+= dist_to_edge*MIN(fabs(edge_angle),fabs(edge_angle-M_PI));
     }
  }

  score4= configCost();

  score1= gpForce_closure_3D_grasp(_contacts, _normals, _mu, contacts.size(), (unsigned int) 6);

  if(isnan(score1)) score1= 0.0;
  if(isnan(score2)) score2= 0.0;
  if(isnan(score3)) score3= 0.0;
  if(isnan(score4)) score4= 0.0;

  if(score1 <= 0) 
  {   
    weight2= weight3= weight4= 0.0;
  }
  else
  {
    weight1= 1.0;
    weight2= 1.0;
    weight3= 1.0;
    weight4= 1.0;
  }
// printf("scores %f %f %f %f\n",score1, score2, score3, score4);

  quality= weight1*score1 + weight2*score2 + weight3*score3 + weight4*score4;

  delete [] _contacts;
  delete [] _normals;
  delete [] _mu;

  return quality;
}


bool gpGrasp::operator == (const gpGrasp &grasp)
{
  if(this==NULL)
  {
    printf("%s: %d: gpGrasp::operator ==: the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  return (ID==grasp.ID);
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
  printf("\t handID: %d (%p)\n", handID, this);
  printf("\t object: %s\n", object_name.c_str());
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
    printf("%s: %d: gpGrasp::print(): the calling instance is NULL.\n",__FILE__,__LINE__);
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
  fprintf(file, "\t handID: %d (%p) \n", handID, this);
  fprintf(file, "\t object: %s\n", object_name.c_str());
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


gpHand_properties::gpHand_properties()
{
  type= GP_HAND_NONE;
  nb_fingers= 0;
  nb_dofs= 0;
}


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

       nb_positions= 2000;
       nb_directions= 10;
       nb_rotations= 6;
       max_nb_grasp_frames= 25000;
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
       for(i=0; i<4; i++)
       {
         q0min[i]=            0;
         q0max[i]=  90*DEGTORAD;
         q1min[i]= -20*DEGTORAD;
         q1max[i]=  20*DEGTORAD;
         q2min[i]= -19*DEGTORAD;
         q2max[i]=  90*DEGTORAD;
         q3min[i]=            0;
         q3max[i]=  70*DEGTORAD;
       }
       //for the thumb:
       q2min[0]= -19*DEGTORAD;
       q2max[0]=  90*DEGTORAD;

       qmin.resize(13);
       qmax.resize(13);
       //thumb:
       qmin[0]=   0.0*DEGTORAD;   qmax[0]= 90.0*DEGTORAD;
       qmin[1]= -20.0*DEGTORAD;   qmax[1]= 20.0*DEGTORAD;
       qmin[2]= -19.0*DEGTORAD;   qmax[2]= 90.0*DEGTORAD;
       qmin[3]=   0.0*DEGTORAD;   qmax[3]= 90.0*DEGTORAD;
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
          Thand_wrist[2][0]=  1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.14;
          Thand_wrist[3][0]=  0.0;   Thand_wrist[3][1]=  0.0;   Thand_wrist[3][2]=  0.0;   Thand_wrist[3][3]=  1.0;
       }

       if(type==GP_SAHAND_LEFT)
       {
          Thand_wrist[0][0]=  0.0;   Thand_wrist[0][1]=  0.0;   Thand_wrist[0][2]= -1.0;   Thand_wrist[0][3]=  0.0;
          Thand_wrist[1][0]=  0.0;   Thand_wrist[1][1]= -1.0;   Thand_wrist[1][2]=  0.0;   Thand_wrist[1][3]=  0.0;
          Thand_wrist[2][0]= -1.0;   Thand_wrist[2][1]=  0.0;   Thand_wrist[2][2]=  0.0;   Thand_wrist[2][3]=  0.14;
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
//        workspace.at(15).setCenter(-0.000000, 0.022663, -0.091911); 
//        workspace.at(15).radius= 0.007871; 
//        workspace.at(16).setCenter(0.016341, 0.116272, -0.013845); 
//        workspace.at(16).radius= 0.007639; 
//        workspace.at(17).setCenter(0.032364, 0.112867, -0.013845); 
//        workspace.at(17).radius= 0.007639; 
//        workspace.at(18).setCenter(-0.032364, 0.112867, -0.013845); 
//        workspace.at(18).radius= 0.007639; 
//        workspace.at(19).setCenter(-0.016341, 0.116272, -0.013845); 
//        workspace.at(19).radius= 0.007639; 
//        workspace.at(20).setCenter(-0.029746, 0.103738, -0.051258); 
//        workspace.at(20).radius= 0.007374; 
//        workspace.at(21).setCenter(0.029746, 0.103738, -0.051258); 
//        workspace.at(21).radius= 0.007374; 
//        workspace.at(22).setCenter(0.008750, 0.049622, -0.068817); 
//        workspace.at(22).radius= 0.007201; 
//        workspace.at(23).setCenter(-0.008750, 0.049622, -0.068817); 
//        workspace.at(23).radius= 0.007201; 
//        workspace.at(24).setCenter(0.000000, -0.020361, -0.115106); 
//        workspace.at(24).radius= 0.006941; 
//        workspace.at(25).setCenter(-0.000000, 0.019585, -0.059740); 
//        workspace.at(25).radius= 0.006792; 
//        workspace.at(26).setCenter(0.000000, -0.019658, -0.085465); 
//        workspace.at(26).radius= 0.006722; 
//        workspace.at(27).setCenter(0.000000, -0.018822, -0.070405); 
//        workspace.at(27).radius= 0.006537; 
//        workspace.at(28).setCenter(-0.000000, 0.080595, -0.089750); 
//        workspace.at(28).radius= 0.006484; 
//        workspace.at(29).setCenter(-0.003954, 0.113227, -0.041408); 
//        workspace.at(29).radius= 0.006484; 
//        workspace.at(30).setCenter(-0.025442, 0.088726, -0.077660); 
//        workspace.at(30).radius= 0.006443; 
//        workspace.at(31).setCenter(0.025442, 0.088726, -0.077660); 
//        workspace.at(31).radius= 0.006443; 
//        workspace.at(32).setCenter(-0.024496, 0.085428, -0.032607); 
//        workspace.at(32).radius= 0.005891; 
//        workspace.at(33).setCenter(0.024496, 0.085428, -0.032607); 
//        workspace.at(33).radius= 0.005891; 
//        workspace.at(34).setCenter(0.028173, 0.098252, -0.064058); 
//        workspace.at(34).radius= 0.005808; 
//        workspace.at(35).setCenter(-0.028173, 0.098252, -0.064058); 
//        workspace.at(35).radius= 0.005808; 
//        workspace.at(36).setCenter(-0.000000, 0.016628, -0.119474); 
//        workspace.at(36).radius= 0.005775; 
//        workspace.at(37).setCenter(-0.004162, 0.119190, -0.007099); 
//        workspace.at(37).radius= 0.005756; 
//        workspace.at(38).setCenter(0.009364, 0.044054, -0.111903); 
//        workspace.at(38).radius= 0.005730; 
//        workspace.at(39).setCenter(-0.009364, 0.044054, -0.111903); 
//        workspace.at(39).radius= 0.005730; 
//        workspace.at(40).setCenter(-0.015714, 0.063024, -0.033048); 
//        workspace.at(40).radius= 0.005464; 

       nb_positions= 500;
       nb_directions= 6;
       nb_rotations= 6;
       max_nb_grasp_frames= 3000;
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

  hand_robot= p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME);
  if(hand_robot!=NULL)
  {
    type= GP_GRIPPER;
  }
  else
  {
    hand_robot= p3d_get_robot_by_name(GP_SAHAND_RIGHT_ROBOT_NAME);
    if(hand_robot!=NULL)
    {
       type= GP_SAHAND_RIGHT;
    }
    else
    {
      hand_robot= p3d_get_robot_by_name(GP_SAHAND_LEFT_ROBOT_NAME);
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

  GLboolean lighting_enable;
  unsigned int i;
  int result= GP_OK;
  GLint line_width;
  float matGL[16];
  p3d_matrix4 Tgrasp_frame_hand_inv, T, T_inv;
  gpSAHandInfo data;

  glGetIntegerv(GL_LINE_WIDTH, &line_width);
  glGetBooleanv(GL_LIGHTING, &lighting_enable);


  p3d_matrix4_to_OpenGL_format(pose, matGL);
  glPushMatrix();
   glMultMatrixf(matGL);
    switch(type)
    {
      case GP_GRIPPER:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
        g3d_draw_frame(Tgrasp_frame_hand_inv, 0.1);

        g3d_set_color_mat(Red, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,-0.5*fingertip_distance,  0.065, fingertip_radius, 20);
        g3d_set_color_mat(Green, NULL);
        g3d_draw_solid_sphere(0.5*min_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(0.5*max_opening,0.5*fingertip_distance, 0.065, fingertip_radius, 20);
        g3d_set_color_mat(Blue, NULL);
        g3d_draw_solid_sphere(-0.5*min_opening,0.0,  0.065, fingertip_radius, 20);
        g3d_draw_solid_sphere(-0.5*max_opening,0.0,  0.065, fingertip_radius, 20);
      break;
      case GP_SAHAND_RIGHT: case GP_SAHAND_LEFT:
        p3d_matInvertXform(Tgrasp_frame_hand, Tgrasp_frame_hand_inv);
        g3d_draw_frame(Tgrasp_frame_hand_inv, 0.1);

        p3d_mat4Mult(Tgrasp_frame_hand_inv, Thand_wrist, T);
        p3d_matInvertXform(T, T_inv);
        g3d_draw_frame(T, 0.1);
        g3d_draw_frame(T_inv, 0.1);
//         p3d_matInvertXform(Thand_wrist, Thand_wrist_inv);
//         g3d_draw_frame(Thand_wrist_inv, 0.1);

        for(i=0; i<4; ++i)
        {
          g3d_draw_frame(Twrist_finger[i], 0.2);
          p3d_matrix4_to_OpenGL_format(Twrist_finger[i], matGL);
          glPushMatrix();
            glMultMatrixf(matGL);
            switch(i)
            { 
              case 0:  g3d_set_color_mat(Red, NULL);  break;
              case 1:  g3d_set_color_mat(Green, NULL);  break;
              case 2:  g3d_set_color_mat(Blue, NULL);  break;
              case 3:  g3d_set_color_mat(Yellow, NULL);  break;
            }
//             gpDraw_SAHfinger_outer_workspace(data, 4*DEGTORAD);
//             for(j=0; j<workspace.size(); ++j)
//             {
//               g3d_draw_solid_sphere(workspace[j].center[0],workspace[j].center[1],workspace[j].center[2], workspace[j].radius, 25);
//             }

//             glPushMatrix();
//               glRotatef(-90, 1.0, 0.0, 0.0);
//               g3d_set_color_mat(Red, NULL);
//               glTranslatef(0, 0, 0.5*length_proxPha);
//               g3d_draw_solid_cylinder(fingertip_radius, length_proxPha, 10);
//               g3d_set_color_mat(Green, NULL);
//               glTranslatef(0, 0, 0.5*(length_proxPha + length_midPha));
//               g3d_draw_solid_cylinder(fingertip_radius, length_midPha, 10);
//               g3d_set_color_mat(Blue, NULL);
//               glTranslatef(0, 0, 0.5*(length_midPha + length_distPha));
//               g3d_draw_solid_cylinder(fingertip_radius, length_distPha, 10);
//             glPopMatrix();
          glPopMatrix();
        }

      break;
      default:
       printf("%s: %d: gpHand_properties::draw(): undefined or unimplemented hand type.\n",__FILE__,__LINE__);
       result= GP_ERROR;
      break;
    }

  glPopMatrix();

  glLineWidth(line_width);
  if(lighting_enable)
  {  glEnable(GL_LIGHTING);  }
  else
  {  glDisable(GL_LIGHTING);  }


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

  
//! Sets the double grasp from two gpGrasp.
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



//! Draws all the contacts of a double grasp.
//! \param length lenght of each friction cone to draw
//! \param nb_slices number of segments of each cone discretization
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDoubleGrasp::draw(double length, int nb_slices)
{  
  if(this==NULL)
  {
    printf("%s: %d: gpDoubleGrasp::draw(): the calling instance is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }

  grasp1.draw(length, nb_slices);
  grasp2.draw(length, nb_slices);

  return GP_OK;
}

