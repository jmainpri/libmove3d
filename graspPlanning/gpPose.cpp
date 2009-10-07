
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <list>

gpPolyhedronFeature::gpPolyhedronFeature()
{
  type= GP_VERTEX;
  polyhedron= NULL;
  vertex_indices[0]= vertex_indices[1]= vertex_indices[2]= 0;
  normals[0][0]= 1.0;     normals[0][1]= 0.0;    normals[0][2]= 0.0;
  normals[1][0]= 1.0;     normals[1][1]= 0.0;    normals[1][2]= 0.0;
  normals[2][0]= 1.0;     normals[2][1]= 0.0;    normals[2][2]= 0.0;
} 

gpPolyhedronFeature::gpPolyhedronFeature(const gpPolyhedronFeature &pf)
{
  unsigned int i;
  type= pf.type;
  polyhedron= pf.polyhedron;
  vertex_indices[0]= pf.vertex_indices[0];
  vertex_indices[1]= pf.vertex_indices[1]; 
  vertex_indices[2]= pf.vertex_indices[2];
  for(i=0; i<3; i++)
  {
    normals[0][i]= pf.normals[0][i];
    normals[1][i]= pf.normals[1][i]; 
    normals[2][i]= pf.normals[2][i];
  }
}  


//! Copy operator of the class gpPolyhedronFeature.
gpPolyhedronFeature & gpPolyhedronFeature::operator=(const gpPolyhedronFeature &pf)
{
  unsigned int i;
  if(this!=&pf)
  { 
    type             = pf.type; 
    polyhedron       = pf.polyhedron;
    vertex_indices[0]= pf.vertex_indices[0];
    vertex_indices[1]= pf.vertex_indices[1]; 
    vertex_indices[2]= pf.vertex_indices[2];
    for(i=0; i<3; i++)
    {
      normals[0][i]= pf.normals[0][i];
      normals[1][i]= pf.normals[1][i]; 
      normals[2][i]= pf.normals[2][i];
    }
  }   

  return *this;
}

//! Default constructor of the class gpPose.
gpPose::gpPose()
{
  plane.normale[0]= plane.normale[1]= 0.0;
  plane.normale[2]= 1.0;
  plane.d= 0.0;
  stability= 0.0;
}  

gpPose::~gpPose()
{
  features.clear();
}  

gpPose::gpPose(const gpPose &pose)
{
  plane.normale[0]= pose.plane.normale[0];
  plane.normale[1]= pose.plane.normale[1];
  plane.normale[2]= pose.plane.normale[2];
  stability  = pose.stability; 
  features= pose.features;
}  

//! Copy operator of the class gpPose.
gpPose & gpPose::operator=(const gpPose &pose)
{
  if(this!=&pose)
  { 
    plane.normale[0]= pose.plane.normale[0];
    plane.normale[1]= pose.plane.normale[1];
    plane.normale[2]= pose.plane.normale[2];
    stability  = pose.stability; 
    features= pose.features;
  }   

  return *this;
}




//! Computes a list of stable poses of the given object.
//! The function first computes the convex hull of the object.
//! Each face of the object's convex hull defines a support polygon.
//! If the orthogonal projection of the object's center of mass on the plane of a face
//! is inside the face, then this face corresponds to a stable pose.
//! One has then to link the hull face to features of the original polyhedron (faces, edges, points).
//! \param object pointer to the object
//! \param cmass object's center of mass (given in the same frame as the vertices of the object polyhedron)
//! \param poseList the computed pose list
//! \return 1 in case of success, 0 otherwise
int gpComputeStablePoses(p3d_obj *object, p3d_vector3 cmass, std::list<gpPose> poseList)
{
  unsigned int i, j, i1, i2, i3;
  double a, dot, d, dmin;
  p3d_vector3 p, p1, p2, p3, ph, pph;
  p3d_vector3 p1p, p2p, p3p, p1p_n, p2p_n, p3p_n, p1p2_n, p2p3_n, p3p1_n;
  p3d_polyhedre *polyhedron= NULL;
  std::vector<double> normal;
  gpConvexHull3D *chull= NULL;
  gpPose pose;

  polyhedron= object->pol[0]->poly;

  chull= new gpConvexHull3D(polyhedron->the_points, polyhedron->nb_points);

  for(i=0; i<chull->hull_faces.size(); i++)
  {
    i1= chull->hull_faces[i][0];
    i2= chull->hull_faces[i][1];
    i3= chull->hull_faces[i][2];

    //compute the orthogonal projection p of the center of mass on the face's plane:
    normal= chull->hull_faces[i].normal();
    a= normal[0]*cmass[0] + normal[1]*cmass[1] + normal[2]*cmass[2] + chull->hull_faces[i].offset();
    p[0]= cmass[0] - a*normal[0];
    p[1]= cmass[1] - a*normal[1];
    p[2]= cmass[2] - a*normal[2];

    //tests if the projection is inside the triangle:
    chull->getFacePoints(i, p1, p2, p3);

    p3d_vectSub(p, p1, p1p);
    p3d_vectNormalize(p1p, p1p_n);
    p3d_vectSub(p, p2, p2p);
    p3d_vectNormalize(p2p, p2p_n);
    p3d_vectSub(p, p3, p3p);
    p3d_vectNormalize(p3p, p3p_n);

    a= acos(p3d_vectDotProd(p1p_n, p2p_n)) + acos(p3d_vectDotProd(p2p_n, p3p_n)) + acos(p3d_vectDotProd(p3p_n, p1p_n));

    if( fabs(a-2*M_PI) > 1e-6 ) // projection is outside the triangle
    { continue; }

    //now find the smallest distance between the gravity center projection and the triangle border:
    p3d_vectSub(p2, p1, p1p2_n);
    p3d_vectNormalize(p1p2_n, p1p2_n);
    p3d_vectSub(p3, p2, p2p3_n);
    p3d_vectNormalize(p2p3_n, p2p3_n);
    p3d_vectSub(p1, p3, p3p1_n);
    p3d_vectNormalize(p3p1_n, p3p1_n);

    // compute the orthogonal projection of p onto each edge of the triangle:
    dot= p3d_vectDotProd(p1p, p1p2_n);
    for(j=0; j<3; j++)
    {  ph[j]= p1[j] + dot*p1p2_n[j];  }
    p3d_vectSub(ph, p, pph);
    dmin= p3d_vectNorm(pph); 

    dot= p3d_vectDotProd(p2p, p2p3_n);
    for(j=0; j<3; j++)
    {  ph[j]= p2[j] + dot*p2p3_n[j];  }
    p3d_vectSub(ph, p, pph);
    d= p3d_vectNorm(pph);
    if(d < dmin) { d= dmin; }

    dot= p3d_vectDotProd(p3p, p3p1_n);
    for(j=0; j<3; j++)
    {  ph[j]= p3[j] + dot*p3p1_n[j];  }
    p3d_vectSub(ph, p, pph);
    d= p3d_vectNorm(pph);
    if(d < dmin) { d= dmin; }

    pose.stability= dmin;
    pose.plane.normale[0]= normal[0];
    pose.plane.normale[1]= normal[1];
    pose.plane.normale[2]= normal[2];

    //gets the pose features:
    for(j=0; j<polyhedron->nb_faces; j++)
    {
    //  polyhedron->the_faces[j].nb_points
    }
  }

  delete chull;

  return 1;
}


