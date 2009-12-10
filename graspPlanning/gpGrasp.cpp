
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
  surface= NULL;
  face= 0;   
  fingerID= 0;
  position[0]= position[1]= position[2]= 0.0;
  normal[0]= normal[1]= normal[2]= 0.0;
  mu= 0.0;
}  


gpContact::gpContact(const gpContact &contact)
{
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
gpContact & gpContact::operator=(const gpContact &contact)
{
  if(this!=&contact)
  { 
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
//! \return 1 in case of success, 0 otherwise
int gpContact::draw(double length, int nb_slices)
{
  #ifdef DEBUG
  if(surface==NULL)
  {
    printf("%s: %d: gpContact::draw((): no surface (p3d_polyhedre) is associated to the contact.\n", __FILE__, __LINE__);
    return 0;
  }
  #endif

  p3d_matrix4 pose;
  GLfloat matGL[16];

  p3d_mat4Copy(p3d_mat4IDENTITY, pose);

  if(surface!=NULL)
  { 
    p3d_get_poly_pos(surface, pose );
  }

  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glDisable(GL_LIGHTING);
  glPushMatrix();
    glMultMatrixf(matGL);
    g3d_drawSphere(position[0], position[1], position[2] , length/10.0, Blue, NULL);
    gpDraw_friction_cone(position, normal, mu, nb_slices, length);
  glPopMatrix();

  glEnable(GL_LIGHTING);

  return 1;
}



//! Default constructor of the class gpGrasp
gpGrasp::gpGrasp()
{
  ID= 0;
  quality= 0;
  p3d_mat4Copy(p3d_mat4IDENTITY, frame);
  polyhedron= NULL;
  object= NULL;
  object_name= "none";
  hand_type= GP_HAND_NONE;
  collision_state= NOT_TESTED;
}

gpGrasp::gpGrasp(const gpGrasp &grasp)
{
  unsigned int i, j;

  ID= grasp.ID;
  quality= grasp.quality;       

  for(i=0; i<4; i++)
  {      
    for(j=0; j<4; j++)
    {  frame[i][j]= grasp.frame[i][j];  }
  }

  polyhedron= grasp.polyhedron;
  object= grasp.object;
  object_name= grasp.object_name;
  hand_type= grasp.hand_type;
  collision_state= grasp.collision_state;

  contacts.resize(grasp.contacts.size());
  for(i=0; i<contacts.size(); i++)
  {  contacts[i]= grasp.contacts[i];  }

  config.resize(grasp.config.size());  
  for(i=0; i<config.size(); i++)
  {  config[i]= grasp.config[i];  }

}

gpGrasp::~gpGrasp()
{
  contacts.clear();
  config.clear();
}

//! Copy operator of the class gpGrasp.
gpGrasp & gpGrasp::operator=(const gpGrasp &grasp)
{
  unsigned int i, j;
  if( this!=&grasp )
  {
    ID= grasp.ID;
    quality= grasp.quality;       

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
    object_name= grasp.object_name;
    hand_type= grasp.hand_type;
    collision_state= grasp.collision_state;

  
    config.resize(grasp.config.size());  
    for(i=0; i<config.size(); i++)
    {  config[i]= grasp.config[i];   }

  }

  return *this;
}



//! Draws all the contacts of a grasp.
//! \param length lenght of each friction cone to draw
//! \param nb_slices number of segments of each cone discretization
//! \return 1 in case of success, 0 otherwise
void gpGrasp::draw(double length, int nb_slices)
{  
  unsigned int i;
  p3d_matrix4 pose;
  GLfloat matGL[16];

  for(i=0; i<contacts.size();i++)
  {
    if(i==0) glColor3f(1.0, 0.0, 0.0);
    if(i==1) glColor3f(0.0, 1.0, 0.0);
    if(i==2) glColor3f(0.0, 0.0, 1.0);
    if(i==3) glColor3f(1.0, 1.0, 0.0);
    if(i==4) glColor3f(1.0, 0.0, 1.0);
    contacts[i].draw(length, nb_slices);
  }
  
  p3d_mat4Copy(p3d_mat4IDENTITY, pose);

  if(object!=NULL)
  {
    p3d_get_obj_pos(object, pose);
  }

  p3d_matrix4_to_OpenGL_format(pose, matGL);

  glDisable(GL_LIGHTING);
  glPushMatrix();
    glMultMatrixf(matGL);
    draw_frame(frame, 4*length);
  glPopMatrix();
}


//! Computes a cost for the given grasping configuration.
//! The biggest it is, the better it is.
double gpGrasp::configCost()
{
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
     default:
        printf("%s: %d: gpGrasp::computeQuality(): unimplemented or unknown hand type.\n", __FILE__, __LINE__);
       return 0;
     break;
   }

}


//! Computes and returns the quality --stability criterion-- of the grasp.
//! For now, the quality is a weighted sum of a "force closure quality criterion"
//! and a score telling how close are the contact normals to the main grasping direction of the hand or gripper
//! (i.e. the (or main) direction along which the hand can exert a force). This last point is still
//! much too coarse. If the grasp does not verify force-closure, its global quality will be kept null.
double gpGrasp::computeQuality()
{
   unsigned int i, j, v1, v2;
   p3d_vector3 graspingDirection; //direction of the grasping force of the hand
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
     case GP_GRIPPER: case GP_SAHAND_RIGHT:
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
        v1= polyhedron->the_edges[triangle.edges[j]].point1 - 1;
        v2= polyhedron->the_edges[triangle.edges[j]].point2 - 1;
        edge_angle= polyhedron->the_edges[triangle.edges[j]].angle;

        dist_to_edge= gpPoint_to_line_segment_distance(contacts[i].position, polyhedron->the_points[v1], polyhedron->the_points[v2]);
        score3+= dist_to_edge*MIN(fabs(edge_angle),fabs(edge_angle-M_PI));
      }
   }

   score4= configCost();

   score1= gpForce_closure_3D_grasp(_contacts, _normals, _mu, contacts.size(), (unsigned int) 6);

   if(isnan(score1)) score1= 1.0;
   if(isnan(score2)) score2= 1.0;
   if(isnan(score3)) score3= 1.0;
   if(isnan(score4)) score4= 1.0;

   if(score1 < 0) 
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
printf("scores %f %f %f %f\n",score1, score2, score3, score4);

   quality= weight1*score1 + weight2*score2 + weight3*score3 + weight4*score4;

   delete [] _contacts;
   delete [] _normals;
   delete [] _mu;


   return quality;
}


bool gpGrasp::operator == (const gpGrasp &grasp)
{
  return (ID==grasp.ID);
}


//! Grasp quality comparison operator.
bool gpGrasp::operator < (const gpGrasp &grasp)
{
  return (quality < grasp.quality) ? true : false;
}

//! Grasp quality comparison operator.
bool gpGrasp::operator > (const gpGrasp &grasp)
{
  return (quality > grasp.quality) ? true : false;
}
 

//! Prints the content of a gpGrasp variable in the standard output.
void gpGrasp::print()
{
  unsigned int i;

  printf("grasp: \n");
  printf("\t ID: %d (%p)\n", ID, this);
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

}


//! Prints the content of a gpGrasp variable in a file.
//! \param filename name of the file
//! \return 1 in case of success, 0 otherwise
int gpGrasp::printInFile(const char *filename)
{
  if(filename==NULL)
  { 
    printf("%s: %d: gpGrasp::printInFile(): input is NULL.\n", __FILE__, __LINE__);
    return 0;
  }

  FILE *file= NULL;
  file= fopen(filename,"a+");
  
  if(file==NULL)
  { 
    printf("%s: %d: gpGrasp::printInFile(): can not open file \"%s\".\n", __FILE__, __LINE__,filename);
    return 0; 
  }

  unsigned int i;

  fprintf(file, "grasp: \n");
  fprintf(file, "\t ID: %d (%p) \n", ID, this);
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

  fclose(file); 

  return 1;
}

