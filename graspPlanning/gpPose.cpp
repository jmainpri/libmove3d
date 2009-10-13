
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
//   features.clear();
}  

gpPose::gpPose(const gpPose &pose)
{
  plane.normale[0]= pose.plane.normale[0];
  plane.normale[1]= pose.plane.normale[1];
  plane.normale[2]= pose.plane.normale[2];
  stability  = pose.stability; 
//   features= pose.features;
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
//     features= pose.features;
  }   

  return *this;
}


//! Tests if a face of a p3d_polyhedre is included in a face of the class used by gpConvexHull3D.
//! The vertex indices of the two faces are supposed to refer to the same vertex array.
//! The ordering of the vertices in each face has no consequence on the comparison.
//! \param pface a reference to a face of a p3d_polyhedre
//! \param gface a reference to a face of a gpConvexHull3D
//! \return true if the faces are identical, false otherwise
// bool gpIsFaceInHullFace(p3d_face &pface, gpFace & gface)
// {
//   bool match;
//   unsigned int i;
//   std::vector<unsigned int> face1;
//   std::vector<unsigned int>::iterator iter;
// 
//   if(pface.nb_points != gface.nbVertices())
//   {
//     return false;
//   }
// 
//   for(i=0; i<pface.nb_points; i++)
//   {
//     face1.push_back(pface.the_indexs_points[i]);
//   }
// 
//   for(i=0; i<gface.nbVertices(); i++)
//   {
//     match= false;
//     for(iter=face1.begin(); iter!=face1.end(); iter++)
//     {
//       if(gface[i]==(*iter))
//       {
//         face1.erase(iter);
//         match= true;
//         break;
//       }
//     }
//     if(!match)
//     {  return false;  }
//   }
// 
//   if(!face1.empty())
//   { return false; }
// 
//   return true;
// }


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
int gpCompute_stable_poses(p3d_obj *object, p3d_vector3 cmass, std::list<gpPose> poseList)
{
  bool stable;
  unsigned int i, j, i1, i2, i3;
  double a,  d, dmin;
  p3d_vector3 p, normal, p1, p2;
//   p3d_vector3 p1p, p2p, p3p, p1p_n, p2p_n, p3p_n, p1p2_n, p2p3_n, p3p1_n;
  p3d_vector3 pp1, pp2, cross;
  p3d_polyhedre *polyhedron= NULL;
  std::vector<double> v;
  gpConvexHull3D *chull= NULL;
  gpPose pose;

  polyhedron= object->pol[0]->poly;

  chull= new gpConvexHull3D(polyhedron->the_points, polyhedron->nb_points);
  chull->compute(false, false);

  for(i=0; i<chull->nbFaces(); i++)
  {
    i1= chull->hull_faces[i][0];
    i2= chull->hull_faces[i][1];
    i3= chull->hull_faces[i][2];

    //compute the orthogonal projection p of the center of mass on the face's plane:
    v= chull->hull_faces[i].normal();
    normal[0]= v[0];
    normal[1]= v[1];
    normal[2]= v[2];

    a= normal[0]*cmass[0] + normal[1]*cmass[1] + normal[2]*cmass[2] + chull->hull_faces[i].offset();
    p[0]= cmass[0] - a*normal[0];
    p[1]= cmass[1] - a*normal[1];
    p[2]= cmass[2] - a*normal[2];

    //tests if the projection is inside the polygon:
    stable= true;
    for(j=0; j<chull->hull_faces[i].nbVertices(); j++)
    {
      if(chull->hull_faces[i].toporient())
      {
        p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][j]], p1);
        p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][(j+1)%chull->hull_faces[i].nbVertices()]], p2);
      }
      else
      {
        p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][j]], p2);
        p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][(j+1)%chull->hull_faces[i].nbVertices()]], p1);
      }
   
      p3d_vectSub(p1, p, pp1);
      p3d_vectSub(p2, p, pp2);

      p3d_vectXprod(pp1, pp2, cross);

      if( p3d_vectDotProd(cross, normal) < 0 ) //projection is outside the support polygon
      {  
        stable= false;
        break;
      }

      d= gpPoint_to_line_segment_distance(p, p1, p2);

     if(d < dmin) { d= dmin; }
    }  
    
    if(stable==false)
    { continue; }

    pose.stability= dmin;
    pose.plane.normale[0]= normal[0];
    pose.plane.normale[1]= normal[1];
    pose.plane.normale[2]= normal[2];
    pose.plane.d= chull->hull_faces[i].offset();
    
    pose.vertices.resize(chull->hull_faces[i].nbVertices());
    for(j=0; j<chull->hull_faces[i].nbVertices(); j++)
    {
      pose.vertices[j]= chull->hull_faces[i][j];
    }
    //gets the pose features:
//     for(j=0; j<polyhedron->nb_faces; j++)
//     {
// //       if(gpCompareFaces(polyhedron->the_faces[j], chull->hull_faces[i]))
// //       {
// // 
// //       }
//     }

  }

  delete chull;

  return 1;
}


