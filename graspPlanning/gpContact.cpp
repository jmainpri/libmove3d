
#include "../graspPlanning/include/gpContact.h"
#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/proto/gp_grasping_utils_proto.h"
#include "../graspPlanning/proto/gp_geometry_proto.h"

//! Default constructor of the class gpContact.
gpContact::gpContact()
{
  ID= 0;
  surface= NULL;
  face= 0;
  fingerID= 0;
  for(int i=0; i<3; ++i)
  {
    position[i]       = 0.0;
    normal[i]         = 0.0;
    forceDirection[i] = 0.0;
    baryCoords[i]     = 0.0;
  }
  curvature= 0;
  mu= 0.3;
}  


gpContact::gpContact(const gpContact &contact)
{
  ID        = contact.ID; 
  surface   = contact.surface; 
  face      = contact.face; 
  fingerID  = contact.fingerID;

  for(int i=0; i<3; i++)
  {
    position[i]       = contact.position[i];
    normal[i]         = contact.normal[i];
    forceDirection[i] = contact.forceDirection[i];
    baryCoords[i]     = contact.baryCoords[i];
  }
  mu= contact.mu;
  curvature= contact.curvature;
  score= contact.score;
}  

gpContact::~gpContact()
{
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
      position[i]       = contact.position[i];
      normal[i]         = contact.normal[i];
      forceDirection[i] = contact.forceDirection[i];
      baryCoords[i]     = contact.baryCoords[i];
    }
    mu= contact.mu;
    curvature= contact.curvature;
    score= contact.score;
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

  p3d_vector3 dir, dir2, normal_inv;
  p3d_matrix4 pose;

  p3d_mat4Copy(p3d_mat4IDENTITY, pose);

  if(surface!=NULL)
  { 
    p3d_get_poly_pos(surface, pose );
  }

  for(int i=0; i<3; ++i)
  {
    dir[i]= position[i] + length*forceDirection[i];
    dir2[i]= position[i] - length*normal[i];
    normal_inv[i]= -normal[i];
  }

  glPushAttrib(GL_LIGHTING_BIT);
  glDisable(GL_LIGHTING);
  glPushMatrix();
    g3d_draw_cylinder(position, dir2, length/20.0, 6);
    gpDraw_friction_cone(position, normal, mu, nb_slices, length);
  //gpDraw_friction_cone(position, normal_inv, mu, nb_slices, length);

    glColor3f(1.0,0.0,0.0);
    g3d_drawSphere(position[0], position[1], position[2], length/10.0);
    g3d_draw_cylinder(position, dir, length/20.0, 6);
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

  if(isnan(c1) || isnan(c2) || isnan(c3))
  {
    curvature= P3D_HUGE;
  }
  else
  {
    curvature= baryCoords[0]*c1 + baryCoords[1]*c2 + baryCoords[2]*c3;
  }

  return GP_OK;
}



//! Computes the distance from the contact to a sharp edge.
//! The function considers the edges of the face the contact is on.
//! If one of the edge is not too "flat", the distance from the contact to the edge is computed and returned
//! \param angleThreshold the edge angle (in radians) above which an edge is regarded as not flat
//! \return the distance to the closest sharp edge, P3D_HUGE if there is no such edge or in case of error
double gpContact::distanceToSharpEdge(double angleThreshold)
{
  unsigned int j, i1, i2;
  double distance;
  p3d_vector3 p1, p2, closestPoint;
  p3d_polyhedre *poly= NULL;
  int face_edges[3];
  float angle;

  distance= P3D_HUGE;
 
  poly= this->surface;
  if(poly==NULL)
  {
    printf("%s: %d: gpContact::distanceToSharpEdge(): the contact has a NULL surface.\n",__FILE__,__LINE__);
    return distance;
  }

  if(this->face >= poly->nb_faces)
  {
    printf("%s: %d: gpContact::distanceToSharpEdge(): inconsistent face index (%d for a poly with %d faces).\n",__FILE__,__LINE__,this->face ,poly->nb_faces);
    return distance;
  }

  // compute the edges of the poly:
  if(poly->areEdgesAndNeighboursUpToDate==FALSE)
  {
    p3d_compute_edges_and_face_neighbours(poly);
  }

  //get the indices of the face edges:
  face_edges[0]= poly->the_faces[this->face].edges[0];
  face_edges[1]= poly->the_faces[this->face].edges[1];
  face_edges[2]= poly->the_faces[this->face].edges[2];

  //for each edges:
  for(j=0; j<3; ++j)
  {
    if(face_edges[j]==-1)
    {  continue;  }

    angle= poly->the_edges[face_edges[j]].angle;

    // if the edge is flat, skip the test
    if( fabs(angle) < angleThreshold )
    {  continue; }

    i1= poly->the_edges[face_edges[j]].point1 -1;
    i2= poly->the_edges[face_edges[j]].point2 -1;
    p3d_vectCopy(poly->the_points[i1], p1);
    p3d_vectCopy(poly->the_points[i2], p2);

    distance= gpPoint_to_line_segment_distance(this->position, p1, p2, closestPoint);
  }

  return distance;
}