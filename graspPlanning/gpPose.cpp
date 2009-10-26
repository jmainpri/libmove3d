
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Move3d-pkg.h"
#include "UserAppli-pkg.h"
#include "GraspPlanning-pkg.h"
#include <list>


gpTriangle::gpTriangle()
{
  i1= i2= i3= 0;
  p1[0]= p1[1]= p1[2]= p2[0]= p2[1] = p2[2]= p3[0]= p3[1]= p3[2]= 0.0;
  description= GP_DESCRIPTION_INDICES;
}


unsigned int gpTriangle::operator [] (unsigned int i) const
{
  if(description==GP_DESCRIPTION_POINTS)
  {
    printf("%s: %d: gpTriangle::operator []: warning: the triangle is supposed to be described by the coordinates of its vertices only.\n",__FILE__,__LINE__);
  }

  switch(i)
  {
    case 0:
      return i1;
    break;
    case 1:
      return i2;
    break;
    case 2:
      return i3;
    break;
    default:
      printf("%s: %d: gpTriangle::operator []: index exceeds vector dimensions.\n",__FILE__,__LINE__);
      return 0;
    break;
  }
}

unsigned int & gpTriangle::operator [] (unsigned int i)
{
  if(description==GP_DESCRIPTION_POINTS)
  {
    description= GP_DESCRIPTION_BOTH;
  }

  switch(i)
  {
    case 0:
      return i1;
    break;
    case 1:
      return i2;
    break;
    case 2:
      return i3;
    break;
    default:
      printf("%s: %d: gpTriangle::operator []: index exceeds vector dimensions.\n",__FILE__,__LINE__);
      return i1;
    break;
  }
}

gpTriangle::gpTriangle(const gpTriangle &triangle)
{
  i1  = triangle.i1; 
  i2  = triangle.i2; 
  i3  = triangle.i3; 
  for(int i=0; i<3; i++)
  {
    p1[i]= triangle.p1[i];
    p2[i]= triangle.p2[i];
    p3[i]= triangle.p3[i];
  }
  description= triangle.description;
}


gpTriangle& gpTriangle::operator=(const gpTriangle &triangle)
{
  if(this!=&triangle)
  { 
    i1  = triangle.i1; 
    i2  = triangle.i2; 
    i3  = triangle.i3; 
    for(int i=0; i<3; i++)
    {
      p1[i]= triangle.p1[i];
      p2[i]= triangle.p2[i];
      p3[i]= triangle.p3[i];
    }
    description= triangle.description;
  }   
  return *this;
}


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
  ID= 0;
  plane.normale[0]= plane.normale[1]= 0.0;
  plane.normale[2]= 1.0;
  plane.d= 0.0;
  center[0]= center[1]= center[2]= 0.0;
  p3d_mat4Copy(p3d_mat4IDENTITY, T);
  stability= 0.0;
  polyhedron= NULL;
  object= NULL;
  object_name= "none";
}  

gpPose::~gpPose()
{
//   features.clear();
}  

gpPose::gpPose(const gpPose &pose)
{
  unsigned int i;

  ID= pose.ID;
  for(i=0; i<3; i++)
  {
    plane.normale[i]= pose.plane.normale[i];
    center[i]= pose.center[i];
    T[i][0]= pose.T[i][0];    T[i][1]= pose.T[i][1];    T[i][2]= pose.T[i][2];    T[i][3]= pose.T[i][3];
  }
  plane.d= pose.plane.d;
  T[3][0]= 0;    T[3][1]= 0;    T[3][2]= 0;    T[3][3]= 1;

  stability  = pose.stability; 
  polyhedron= pose.polyhedron;
  object= pose.object;
  object_name= pose.object_name;
  contacts.resize(pose.contacts.size());
  for(i=0; i<pose.contacts.size(); i++)
  {
    contacts[i]= pose.contacts[i];
  }
//   features= pose.features;
}  

//! Copy operator of the class gpPose.
gpPose & gpPose::operator=(const gpPose &pose)
{
  unsigned int i;
 
  if(this!=&pose)
  { 
    ID= pose.ID;
    for(i=0; i<3; i++)
    {
      plane.normale[i]= pose.plane.normale[i];
      center[i]= pose.center[i];
      T[i][0]= pose.T[i][0];    T[i][1]= pose.T[i][1];    T[i][2]= pose.T[i][2];    T[i][3]= pose.T[i][3];
    }
    plane.d= pose.plane.d;
    T[3][0]= 0;    T[3][1]= 0;    T[3][2]= 0;    T[3][3]= 1;
    stability  = pose.stability; 
//     features= pose.features;
    polyhedron= pose.polyhedron;
    object= pose.object;
    object_name= pose.object_name;
    contacts.resize(pose.contacts.size());
    for(i=0; i<pose.contacts.size(); i++)
    {
      contacts[i]= pose.contacts[i];
    }
  }   

  return *this;
}

//! Pose stability comparison operator.
bool gpPose::operator < (const gpPose &pose)
{
  return (stability < pose.stability) ? true : false;
}

//! Pose stability comparison operator.
bool gpPose::operator > (const gpPose &pose)
{
  return (stability > pose.stability) ? true : false;
}


int gpPose::print()
{

  unsigned int i;

  printf("pose: \n");
  printf("\t ID: %d (%p)\n", ID, this);
  printf("\t object: %s\n", object_name.c_str());
  printf("\t stability: %f \n", stability);
//   printf("\t frame: [ %f %f %f %f \n", frame[0][0], frame[0][1], frame[0][2], frame[0][3]);
//   printf("\t          %f %f %f %f \n", frame[1][0], frame[1][1], frame[1][2], frame[1][3]);
//   printf("\t          %f %f %f %f \n", frame[2][0], frame[2][1], frame[2][2], frame[2][3]);
//   printf("\t          %f %f %f %f ] \n", frame[3][0], frame[3][1], frame[3][2], frame[3][3]);
  printf("\t nb_contacts: %d \n", contacts.size());
  printf("\t contacts:\n");

  for(i=0; i<contacts.size(); i++)
  {
    printf("\t\t contact %d:\n", i);
    printf("\t\t\t position: [%f %f %f]\n",contacts[i].position[0],contacts[i].position[1],contacts[i].position[2]);
    printf("\t\t\t normal:   [%f %f %f]\n",contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]);
  }

  return 1;
}

int gpPose::draw(double length)
{
  unsigned int i, j, n, i1, i2;
  double d, step;
  p3d_vector3 diff;
  p3d_matrix4 Tpose, T2;
  double color[4];
  GLfloat matGL[16];

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT);
  
  glColor3f(1, 0, 0);
  for(i=0; i<contacts.size(); i++)
  {
    contacts[i].draw(length, 10);
  }

  glDisable(GL_LIGHTING);

  p3d_mat4Copy(p3d_mat4IDENTITY, Tpose);

//   if(object!=NULL)
//   {
//     p3d_get_obj_pos(object, Tpose);
//   }

  p3d_matMultXform(Tpose, T, T2);
  //p3d_matrix4_to_OpenGL_format(Tpose, matGL);
  p3d_matrix4_to_OpenGL_format(T2, matGL);


  glLineWidth(4);

  glPushMatrix();
    glMultMatrixf(matGL);

    g3d_rgb_from_int(ID, color);
    g3d_set_color_mat(Any, color);
    if(polyhedron!=NULL)
    {   draw_p3d_polyhedre(polyhedron);  }

    //display each contact:
    glColor3f(0, 0, 1);
    glBegin(GL_LINE_LOOP);
    for(i=0; i<contacts.size(); i++)
    {
      glVertex3f(contacts[i].position[0], contacts[i].position[1], contacts[i].position[2]);
    }
    glEnd();

    //display the normal at center point:
    glColor3f(1, 0, 1);
    glBegin(GL_LINES);
      glVertex3f(center[0], center[1], center[2]);
      glVertex3f(center[0]+length*plane.normale[0], center[1]+length*plane.normale[1], center[2]+length*plane.normale[2]);
    glEnd();

    //display the pose surface as lines starting from the center point and going to points of the discretized contour:
    glColor3f(0, 0, 1);
    g3d_rgb_from_hue(stability, color);
    glColor3f(color[0], color[1], color[2]);
    glColor3f(stability,stability,stability);
    glBegin(GL_LINES);
    for(i=0; i<contacts.size(); i++)
    {
        i1= i;
        if(i < contacts.size()-1)
        {  i2= i1 + 1;  }
        else
        {  i2= 0;  }
  
        p3d_vectSub(contacts[i2].position, contacts[i1].position, diff);
        d= p3d_vectNorm(diff); 
        p3d_vectScale(diff, diff, 1/d);
        n= (unsigned int) (d/(0.3*length));
        step = d/((double) n); 
  
        for(j=0; j<n; j++)
        {
          glVertex3f(center[0], center[1], center[2]);
          glVertex3f(contacts[i1].position[0]+j*step*diff[0],contacts[i1].position[1]+j*step*diff[1],contacts[i1].position[2]+j*step*diff[2]);        
        }
    }
    glEnd();  


  glPopMatrix();

  glPopAttrib();

  return 1;
}

void gpPose::setPosition(double x, double y, double z)
{
  T[3][0]= x;
  T[3][1]= y;
  T[3][2]= z;
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
int gpCompute_stable_poses(p3d_obj *object, p3d_vector3 cmass, std::list<gpPose> &poseList)
{
  bool stable;
  unsigned int i, j, index;
  double threshold= 0.003;
  double a,  d, dmin, theta, max;
  p3d_vector3 proj, normal, p1, p2;
  p3d_vector3 pp1, pp2, cross;
  p3d_vector3 axis, Zaxis, new_center, t;
  p3d_polyhedre *polyhedron= NULL;
  std::vector<double> v;
  gpConvexHull3D *chull= NULL;
  gpPose pose;
  std::list<gpPose>::iterator iter;

  polyhedron= object->pol[0]->poly;

  chull= new gpConvexHull3D(polyhedron->the_points, polyhedron->nb_points);
  chull->compute(false, threshold, false);
//   chull->print();

  for(i=0; i<chull->nbFaces(); i++)
  {
    //compute the orthogonal projection p of the center of mass on the face's plane:
    v= chull->hull_faces[i].normal();
    normal[0]= v[0];
    normal[1]= v[1];
    normal[2]= v[2];

    a= normal[0]*cmass[0] + normal[1]*cmass[1] + normal[2]*cmass[2] + chull->hull_faces[i].offset();
    proj[0]= cmass[0] - a*normal[0];
    proj[1]= cmass[1] - a*normal[1];
    proj[2]= cmass[2] - a*normal[2];

    //tests if the projection is inside the polygon:
    stable= true;
    for(j=0; j<chull->hull_faces[i].nbVertices(); j++)
    {
      p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][j]], p1);

      if(j < chull->hull_faces[i].nbVertices()-1)
      {  p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][j+1]], p2);  }
      else
      {  p3d_vectCopy(polyhedron->the_points[chull->hull_faces[i][0]], p2);  }
 

      p3d_vectSub(p1, proj, pp1);
      p3d_vectSub(p2, proj, pp2);

      p3d_vectNormalize(pp1, pp1);
      p3d_vectNormalize(pp2, pp2);

      p3d_vectXprod(pp1, pp2, cross);
      p3d_vectNormalize(cross, cross);

      if( p3d_vectDotProd(cross, normal) < 0 ) //projection is outside the support polygon
      {  
        stable= false;
        break;
      }

      d= gpPoint_to_line_segment_distance(proj, p1, p2);

      if(j==0) { dmin= d;}
      else if(d < dmin)
      { dmin= d; }
    }
    
    if(stable==false)// || dmin < threshold)
    { continue; }

    pose.stability= dmin;
    pose.plane.normale[0]= normal[0];
    pose.plane.normale[1]= normal[1];
    pose.plane.normale[2]= normal[2];
    pose.plane.d= chull->hull_faces[i].offset();
    pose.center[0]= proj[0];
    pose.center[1]= proj[1];
    pose.center[2]= proj[2];
    pose.polyhedron = polyhedron;
    pose.object     = object;
    pose.object_name= object->name;
    
    pose.contacts.resize(chull->hull_faces[i].nbVertices());

    for(j=0; j<chull->hull_faces[i].nbVertices(); j++)
    {
      index= chull->hull_faces[i][j];
      pose.contacts[j].surface= polyhedron;
      p3d_vectCopy(polyhedron->the_points[index], pose.contacts[j].position);
    
      // reproject the contact points onto the support plane (convex hull faces may not be perfectly planar
      // because of facet post-merging):
      a= normal[0]*pose.contacts[j].position[0] + normal[1]*pose.contacts[j].position[1] + normal[2]*pose.contacts[j].position[2] + chull->hull_faces[i].offset();
      pose.contacts[j].position[0]= pose.contacts[j].position[0] - a*normal[0];
      pose.contacts[j].position[1]= pose.contacts[j].position[1] - a*normal[1];
      pose.contacts[j].position[2]= pose.contacts[j].position[2] - a*normal[2];

      p3d_vectCopy(normal, pose.contacts[j].normal);
      pose.contacts[j].mu= 0.2;
    }

    //compute the transformation so that the support plane normal is vertical and its center is 
    // at position (0,0,0): 
    Zaxis[0]= 0.0;
    Zaxis[1]= 0.0;
    Zaxis[2]= -1.0;
    new_center[0]= 0.0;
    new_center[1]= 0.0;
    new_center[2]= 0.0;
    p3d_vectXprod(pose.plane.normale, Zaxis, axis);
    theta= acos(p3d_vectDotProd(pose.plane.normale, Zaxis));
    p3d_mat4Rot(pose.T, axis, theta);
    p3d_vector3 v;
    p3d_xformVect(pose.T, pose.plane.normale, v);
    p3d_xformVect(pose.T, pose.center, v);
    p3d_vectSub(new_center, v, t);

    pose.T[0][3]= t[0];
    pose.T[1][3]= t[1];
    pose.T[2][3]= t[2];
    p3d_xformPoint(pose.T, pose.center, v);
    poseList.push_back(pose);
  }

  poseList.sort(); //sort from the smallest to the biggest stability
  poseList.reverse(); //reverse the order of the elements in the list

  max= poseList.front().stability;

  i=1;
  for(iter= poseList.begin(); iter!=poseList.end(); iter++)
  {  
    (*iter).ID= i++;
    (*iter).stability/= max;
  }

  delete chull;

  return 1;
}


int gpFind_poses_on_object(p3d_obj *object, p3d_obj *support, std::list<gpPose> &poseListIn, double translationStep, unsigned int nbOrientations, std::list<gpPose> &poseListOut)
{
  bool outside;
  unsigned int i, j, k, nb_triangles, nb_samples;
  int result;
  p3d_index *face_indices= NULL;
  double angle;
  p3d_vector3 normal;
  p3d_vector3 *points= NULL, *trisamples= NULL;
  p3d_polyhedre *polyh= NULL;
  gpTriangle triangle;
  gpPose pose;
  std::list<gpTriangle> htris;
  std::list<gpTriangle>::iterator iterT1, iterT2;
  std::list<gpPose>::iterator iterP;
  std::list<gpVector3D> sampleList;
  std::list<gpVector3D>::iterator iterS;
  #ifdef PQP
  pqp_triangle *triangles= NULL;
  #endif

  poseListOut.clear();

  //first eliminate all the faces of the support that are not horizontal and triangulate the non-triangular ones:
  for(i=0; i<(unsigned int) support->np; i++)
  {
    polyh= support->pol[i]->poly;
    poly_build_planes(polyh);
    points= polyh->the_points;
    for(j=0; j<polyh->nb_faces; j++)
    {
      p3d_xformVect(support->pol[i]->pos_rel_jnt, polyh->the_faces[j].plane->normale, normal);
      p3d_vectNormalize(normal, normal);
      face_indices= polyh->the_faces[j].the_indexs_points;

      angle= fabs( (180.0/M_PI)*acos(normal[2]) );
      if( (normal[2]<0) || angle>5 )
      {  continue;  }
      if(polyh->the_faces[j].nb_points==3)
      {
         p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[face_indices[0] - 1], triangle.p1);
         p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[face_indices[1] - 1], triangle.p2);
         p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[face_indices[2] - 1], triangle.p3);
         triangle.description= GP_DESCRIPTION_POINTS;
         htris.push_back(triangle);
      }
      else
      {
        #ifndef PQP
        printf("%s: %d: gpFind_poses_on_object(): some functions in p3d_pqp are needed to deal with non triangular faces.\n", __FILE__,__LINE__);
        #endif
        triangles= pqp_triangulate_face(polyh, j, &nb_triangles);
        for(k=0; k<nb_triangles; k++)
        {
           p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[triangles[k][0]], triangle.p1);
           p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[triangles[k][1]], triangle.p2);
           p3d_xformPoint(support->pol[i]->pos_rel_jnt, points[triangles[k][2]], triangle.p3);
           htris.push_back(triangle);
        }
        free(triangles);
      }
    }
  }

 //display the horizontal triangles:
//   glBegin(GL_TRIANGLES);
//     for(iterT1=htris.begin(); iterT1!=htris.end(); iterT1++)
//     {
//       glVertex3f( (*iterT1).p1[0], (*iterT1).p1[1], (*iterT1).p1[2] );
//       glVertex3f( (*iterT1).p2[0], (*iterT1).p2[1], (*iterT1).p2[2] );
//       glVertex3f( (*iterT1).p3[0], (*iterT1).p3[1], (*iterT1).p3[2] );
//     }
//   glEnd();

  double theta;
  p3d_vector3 Zaxis, contact;
  p3d_matrix4 T, T2;
  Zaxis[0]= 0;
  Zaxis[1]= 0;
  Zaxis[2]= 1;


  while(poseListIn.size()>3)
  { poseListIn.pop_back();  }
//   poseListIn.pop_front(); 

  for(iterT1=htris.begin(); iterT1!=htris.end(); iterT1++)
  {
    //sample each horizontal triangle surface:
    trisamples= gpSample_triangle_surface((*iterT1).p1,(*iterT1).p2,(*iterT1).p3, translationStep, &nb_samples);

    if(trisamples==NULL)
    {   continue;   }

    for(i=0; i<nb_samples; i++)
    {
      //for each possible orientation around the vertical axis:
      for(j=0; j<nbOrientations; j++)
      {
        theta= j*2*M_PI/((double) nbOrientations);
        p3d_mat4Rot(T, Zaxis, theta);
        T[0][3]= trisamples[i][0];
        T[1][3]= trisamples[i][1];
        T[2][3]= trisamples[i][2] + 0.005;

        //for each initial pose:
        for(iterP=poseListIn.begin(); iterP!=poseListIn.end(); iterP++)
        {
           //place at the current position and orientation:
           p3d_matMultXform(T, (*iterP).T, T2);

           //compute the positions of each contact point in world frame coordinates: 
           for(k=0; k<(*iterP).contacts.size(); k++)
           { 
              p3d_xformPoint(T2, (*iterP).contacts[k].position, contact);
              outside= false;
              // test if the contact is included in one of the horizontal triangles:
              for(iterT2=htris.begin(); iterT2!=htris.end(); iterT2++)
              {
                result= gpIs_point_in_triangle(contact, (*iterT2).p1, (*iterT2).p2, (*iterT2).p3) ;

                if(result==1)
                {  break;  }
              }
              if(iterT2==htris.end()) // contact point is outside all the triangles
              {
                outside= true;
                break;
              }
           }
           if(outside==true) // a contact was outside the support triangles
           {  continue;  }
           if(outside==false) // a contact was outside the support triangles
           {  T2[2][3]+= 0.001;  }
           
           #ifdef PQP
           pqp_activate_object_collision(object);
           pqp_set_obj_pos(object, T2, 0);
// p3d_mat4Print(T2, "T2");
           if(pqp_obj_environment_collision_test(object))
           { continue; }
           #else
             printf("%s: %d: gpFind_poses_on_object(): some functions in p3d_pqp are needed for specific collision tests.\n", __FILE__,__LINE__);
           #endif
 
           pose= (*iterP);
           pose.theta= theta;
           p3d_mat4Copy(T2, pose.T);
           poseListOut.push_back(pose);
        }
      
      }
    }

    free(trisamples);
    trisamples= NULL; 
  }

//   poseListOut.sort(); //sort from the smallest to the biggest stability
//   poseListOut.reverse(); //reverse the order of the elements in the list

  i=1;
  for(iterP= poseListOut.begin(); iterP!=poseListOut.end(); iterP++)
  {  
    (*iterP).ID= i++;
  }


  return 1;
}

/*
int gpSample_horizontal_faces(p3d_obj *object, double step, std::list<gpVector3D> &sampleList)
{
  unsigned int i, j, k, i1, i2, i3, nb_samples, nb_triangles;
  double angle;
  p3d_vector3 normal, p, *trisamples= NULL;
  p3d_matrix4 pose;
  p3d_polyhedre *poly= NULL;
  gpVector3D sample;

  #ifdef PQP
  pqp_triangle *triangles= NULL;
  #endif

  if(object==NULL)
  {
    printf("%s: %d: gpSample_horizontal_faces(): object is NULL.\n", __FILE__,__LINE__);
    return 0;
  }


  poly= object->pol[0]->poly;
  poly_build_planes(poly);

  if( step <= 0 )
  {
    printf("%s: %d: gpSample_horizontal_faces(): the \"step\" argument must be > 0.\n", __FILE__,__LINE__);
    return 0;
  }


  p3d_get_obj_pos(object, pose);

  for(i=0; i<poly->nb_faces; i++)
  {
    p3d_xformVect(pose, poly->the_faces[i].plane->normale, normal);
    p3d_vectNormalize(normal, normal);

    angle= fabs( (180.0/M_PI)*acos(normal[2]) );
    if( (normal[2]<0) || angle>5 )
    {  continue;  }

    if(poly->the_faces[i].nb_points==3) // triangle face
    {
        i1= poly->the_faces[i].the_indexs_points[0] - 1;
        i2= poly->the_faces[i].the_indexs_points[1] - 1;
        i3= poly->the_faces[i].the_indexs_points[2] - 1;
  
        trisamples= gpSample_triangle_surface(poly->the_points[i1],poly->the_points[i2],poly->the_points[i3], step, &nb_samples);
  
        if(trisamples!=NULL)
        {
          for(j=0; j<nb_samples; j++)
          {
            p3d_xformPoint(pose, trisamples[j], p); // object frame -> world frame
            sample[0]= p[0];
            sample[1]= p[1];
            sample[2]= p[2];
            sampleList.push_back(sample);
          }
          free(trisamples);
        }
    }
    else // non triangular face
    {
        #ifndef PQP
        printf("%s: %d: gpSample_horizontal_faces(): some functions in p3d_pqp are needed to deal with non triangular faces.\n", __FILE__,__LINE__);
        #endif
        triangles= pqp_triangulate_face(poly, i, &nb_triangles);
    
        if(triangles!=NULL)
        {
           for(j=0; j<nb_triangles; j++)
           {
              i1= triangles[j][0];
              i2= triangles[j][1];
              i3= triangles[j][2];
              trisamples= gpSample_triangle_surface(poly->the_points[i1],poly->the_points[i2],poly->the_points[i3], step, &nb_samples);
    
              if(trisamples!=NULL)
              {
                for(k=0; k<nb_samples; k++)
                {
                  p3d_xformPoint(pose, trisamples[k], p); // object frame -> world frame
                  sample[0]= p[0];
                  sample[1]= p[1];
                  sample[2]= p[2];
                  sampleList.push_back(sample);
                }
                free(trisamples);
              }
           }
        }
    }

  }


  return 1;
}
*/

