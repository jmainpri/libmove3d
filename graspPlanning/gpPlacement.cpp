
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
    printf("%s: %d: gpTriangle::operator []: warning: the triangle is supplacementd to be described by the coordinates of its vertices only.\n",__FILE__,__LINE__);
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


gpTriangle& gpTriangle::operator = (const gpTriangle &triangle)
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


//! Default constructor of the class gpPlacement.
gpPlacement::gpPlacement()
{
  ID= 0;
  plane.normale[0]= plane.normale[1]= 0.0;
  plane.normale[2]= 1.0;
  plane.d= 0.0;
  center[0]= center[1]= center[2]= 0.0;
  position[0]= position[1]= position[2]= 0.0;
  theta= 0.0;
  p3d_mat4Copy(p3d_mat4IDENTITY, T);
  stability  = 0.0;
  clearance  = 0.0;
  polyhedron = NULL;
  object     = NULL;
  object_name= "none";
}  

gpPlacement::~gpPlacement()
{
}  

gpPlacement::gpPlacement(const gpPlacement &placement)
{
  unsigned int i;

  ID= placement.ID;

  for(i=0; i<3; i++)
  {
    plane.normale[i]= placement.plane.normale[i];
    center[i]= placement.center[i];
    position[i]= placement.position[i];
    T[i][0]= placement.T[i][0];    T[i][1]= placement.T[i][1];    T[i][2]= placement.T[i][2];    T[i][3]= placement.T[i][3];
  }
  plane.d= placement.plane.d;
  T[3][0]= 0;    T[3][1]= 0;    T[3][2]= 0;    T[3][3]= 1;
  theta= placement.theta;

  stability   = placement.stability; 
  clearance   = placement.clearance; 
  polyhedron  = placement.polyhedron;
  object      = placement.object;
  object_name = placement.object_name;
  contacts.resize(placement.contacts.size());
  for(i=0; i<placement.contacts.size(); i++)
  {
    contacts[i]= placement.contacts[i];
  }
}  

//! Copy operator of the class gpPlacement.
gpPlacement & gpPlacement::operator = (const gpPlacement &placement)
{
  unsigned int i;
 
  if(this!=&placement)
  { 
    ID= placement.ID;

    for(i=0; i<3; i++)
    {
      plane.normale[i]= placement.plane.normale[i];
      center[i]= placement.center[i];
      position[i]= placement.position[i];
      T[i][0]= placement.T[i][0];    T[i][1]= placement.T[i][1];    T[i][2]= placement.T[i][2];    T[i][3]= placement.T[i][3];
    }
    plane.d= placement.plane.d;
    T[3][0]= 0;    T[3][1]= 0;    T[3][2]= 0;    T[3][3]= 1;
    theta= placement.theta;
    stability   = placement.stability; 
    clearance   = placement.clearance; 
    polyhedron  = placement.polyhedron;
    object      = placement.object;
    object_name = placement.object_name;
    contacts.resize(placement.contacts.size());
    for(i=0; i<placement.contacts.size(); i++)
    {
      contacts[i]= placement.contacts[i];
    }
  }   

  return *this;
}

//! Placement comparison operator.
bool gpPlacement::operator < (const gpPlacement &placement)
{
  double dstab, dclear;

  dstab = fabs(stability - placement.stability);
  dclear= fabs(clearance - placement.clearance);

  if(dstab > dclear) 
  {
    return (stability < placement.stability) ? true : false;
  }
  else
  {
    return (clearance < placement.clearance) ? true : false;
  }
}

//! Comparison function of the stability scores of two placements.
bool gpCompareStability(const gpPlacement &place1, const gpPlacement &place2)
{
  return (place1.stability > place2.stability) ? true : false;
}

//! Comparison function of the clearance scores of two placements.
bool gpCompareClearance(const gpPlacement &place1, const gpPlacement &place2)
{
  return (place1.clearance > place2.clearance) ? true : false;
}

//! Prints some info about the placement;
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpPlacement::print()
{
  unsigned int i;

  printf("placement: \n");
  printf("\t ID: %d (%p)\n", ID, this);
  printf("\t object: %s\n", object_name.c_str());
  printf("\t stability: %f \n", stability);
  printf("\t clearance: %f \n", clearance);
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

  return GP_OK;
}

//! Draws the placement (object and contacts points);
//! \param length length of the friction cones of the placement contacts
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpPlacement::draw(double length)
{
  unsigned int i, j, n, i1, i2;
  double d, step;
  p3d_vector3 diff;
  double color[4];
  GLfloat matGL[16];
  p3d_vector3 zAxis= {0.0, 0.0, 1.0};
  p3d_matrix4 T1, T2;

  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_LINE_BIT);
  
  glColor3f(1, 0, 0);
  for(i=0; i<contacts.size(); i++)
  {
    contacts[i].draw(length, 10);
  }

  glDisable(GL_LIGHTING);


  // place the object to its placement placement:
  p3d_mat4Rot(T1, zAxis, theta);

  T1[0][3]= position[0];
  T1[1][3]= position[1];
  T1[2][3]= position[2];

  p3d_mat4Mult(T1, T, T2);

  p3d_matrix4_to_OpenGL_format(T2, matGL);

  glLineWidth(4);

  glPushMatrix();
    glMultMatrixf(matGL);

    g3d_rgb_from_int(ID, color);
    g3d_set_color(Any, color);
    if(polyhedron!=NULL)
    {   g3d_draw_p3d_polyhedre(polyhedron);  }

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

/*
    //display the placement surface as lines starting from the center point and going to points of the discretized contour:
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
*/

  glPopMatrix();

  glPopAttrib();

  return GP_OK;
}

//! @ingroup stablePlacementComputation
//! Computes a list of stable placements (on a plane) of the given object.
//! The function first computes the convex hull of the object.
//! Each face of the object's convex hull defines a support polygon.
//! If the orthogonal projection of the object's center of mass on the plane of a face
//! is inside the face, then this face corresponds to a stable placement.
//! \param object pointer to the object (a freeflyer robot whose only the first polyhedron of the first body will be considered)
//! \param placementList the computed placement list
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpCompute_stable_placements(p3d_rob *object, std::list<gpPlacement> &placementList)
{
  #ifdef GP_DEBUG
  if(object==NULL)
  {
    printf("%s: %d: gpCompute_stable_placements: input object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  bool stable;
  unsigned int i, j, index;
  double threshold= 0.003;
  double a,  d, dmin, dot, alpha, max;
  p3d_vector3 cmass, proj, normal, p1, p2;
  p3d_vector3 pp1, pp2, cross, closest;
  p3d_vector3 axis, xAxis, zAxis, new_center, t;
  p3d_polyhedre *polyhedron= NULL;
  std::vector<double> v;
  gpConvexHull3D *chull= NULL;
  gpPlacement placement;
  std::list<gpPlacement>::iterator iter;

  polyhedron= object->o[0]->pol[0]->poly;
  gpCompute_mass_properties(polyhedron);
  p3d_vectCopy(polyhedron->cmass, cmass);

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

      d= gpPoint_to_line_segment_distance(proj, p1, p2, closest);

      if(j==0) { dmin= d;}
      else if(d < dmin)
      { dmin= d; }
    }
    
    if(stable==false)// || dmin < threshold)
    { continue; }

    placement.stability= dmin;
    placement.plane.normale[0]= normal[0];
    placement.plane.normale[1]= normal[1];
    placement.plane.normale[2]= normal[2];
    placement.plane.d= chull->hull_faces[i].offset();
    placement.center[0]= proj[0];
    placement.center[1]= proj[1];
    placement.center[2]= proj[2];
    placement.polyhedron = polyhedron;
    placement.object     = object;
    placement.object_name= object->name;
    
    placement.contacts.resize(chull->hull_faces[i].nbVertices());

    for(j=0; j<chull->hull_faces[i].nbVertices(); j++)
    {
      index= chull->hull_faces[i][j];
      placement.contacts[j].surface= polyhedron;
      p3d_vectCopy(polyhedron->the_points[index], placement.contacts[j].position);
    
      // reproject the contact points onto the support plane (convex hull faces may not be perfectly planar
      // because of facet post-merging):
      a= normal[0]*placement.contacts[j].position[0] + normal[1]*placement.contacts[j].position[1] + normal[2]*placement.contacts[j].position[2] + chull->hull_faces[i].offset();
      placement.contacts[j].position[0]= placement.contacts[j].position[0] - a*normal[0];
      placement.contacts[j].position[1]= placement.contacts[j].position[1] - a*normal[1];
      placement.contacts[j].position[2]= placement.contacts[j].position[2] - a*normal[2];

      p3d_vectCopy(normal, placement.contacts[j].normal);
      placement.contacts[j].mu= 0.2;
    }

    // compute the transformation so that the support plane normal is vertical and its center is 
    // at position (0,0,0): 
    xAxis[0]= 1.0;
    xAxis[1]= 0.0;
    xAxis[2]= 0.0;
    zAxis[0]= 0.0;
    zAxis[1]= 0.0;
    zAxis[2]= -1.0;
    new_center[0]= 0.0;
    new_center[1]= 0.0;
    new_center[2]= 0.0;

    dot= p3d_vectDotProd(placement.plane.normale, zAxis);
    if( fabs(fabs(dot) - 1) < 1e-6 )
    {
      if(dot > 0)
      { p3d_mat4Copy(p3d_mat4IDENTITY, placement.T); }
      else
      {
        p3d_mat4Rot(placement.T, xAxis, M_PI);
      }
    }
    else
    {
      alpha= acos(p3d_vectDotProd(placement.plane.normale, zAxis));
      p3d_vectXprod(placement.plane.normale, zAxis, axis);
      p3d_mat4Rot(placement.T, axis, alpha);
    }

    p3d_vector3 v;
    p3d_xformVect(placement.T, placement.plane.normale, v);
    p3d_xformVect(placement.T, placement.center, v);
    p3d_vectSub(new_center, v, t);

    placement.T[0][3]= t[0];
    placement.T[1][3]= t[1];
    placement.T[2][3]= t[2];

    p3d_xformPoint(placement.T, placement.center, v);

    placementList.push_back(placement);
  }

  placementList.sort(gpCompareStability); //sort from the smallest to the biggest stability
  placementList.reverse(); //reverse the order of the elements in the list

  max= placementList.front().stability;

  // normalize the stability score:
  i=1;
  for(iter= placementList.begin(); iter!=placementList.end(); iter++)
  {  
    (*iter).ID= i++;
    (*iter).stability/= max;
  }

  delete chull;

  return GP_OK;
}

//! @ingroup stablePlacementComputation
//! Computes a list of stable placements of an object onto another object.
//! The function receives an initial list of stable placements of the object on infinite planes.
//! The almost horizontal faces of the support are sampled according to specified steps 
//! and each placement of the input list is then tested for each sample.
//! \param object pointer to the object
//! \param support pointer to the object (the support) we want to place the first object on
//! \param placementListIn the input placement list
//! \param translationStep the translation step of the discretization of the horizontal faces of the support
//! \param nbOrientations the number of orientations (around vertical axis) that will be tested to place the object
//! \param placementListOut the output placement list
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFind_placements_on_object(p3d_rob *object, p3d_rob *support, std::list<gpPlacement> placementListIn, double translationStep, unsigned int nbOrientations, std::list<gpPlacement> &placementListOut)
{
  #ifdef GP_DEBUG
  if(object==NULL)
  {
    printf("%s: %d: gpFind_placements_on_object: input object is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  if(support==NULL)
  {
    printf("%s: %d: gpFind_placements_on_object: input support is NULL.\n",__FILE__,__LINE__);
    return GP_ERROR;
  }
  #endif

  bool outside;
  unsigned int i, j, k, nb_triangles, nb_samples;
  int result;
  p3d_index *face_indices= NULL;
  double angle;
  p3d_vector3 normal;
  p3d_vector3 *points= NULL, *trisamples= NULL;
  p3d_matrix4 Tsupport;
  p3d_polyhedre *polyh= NULL;
  gpTriangle triangle;
  gpPlacement placement;
  std::list<gpTriangle> htris;
  std::list<gpTriangle>::iterator iterT1, iterT2;
  std::list<gpPlacement>::iterator iterP;
  std::list<gpVector3D> sampleList;
  std::list<gpVector3D>::iterator iterS;

  placementListOut.clear();
 
  //first eliminate all the faces of the support that are not horizontal and triangulate the non-triangular ones
  //and store the other ones in world coordinates:
  for(i=0; i<(unsigned int) support->o[0]->np; ++i)
  {
    p3d_matMultXform(support->o[0]->jnt->abs_pos, support->o[0]->pol[i]->pos_rel_jnt, Tsupport);

    polyh= support->o[0]->pol[i]->poly;
    poly_build_planes(polyh);
    points= polyh->the_points;
    for(j=0; j<polyh->nb_faces; j++)
    {
      p3d_xformVect(Tsupport, polyh->the_faces[j].plane->normale, normal);
      p3d_vectNormalize(normal, normal);
      face_indices= polyh->the_faces[j].the_indexs_points;

      angle= fabs( (180.0/M_PI)*acos(normal[2]) );
      if( (normal[2]<0) || angle>5 )
      {  continue;  }
      if(polyh->the_faces[j].nb_points==3)
      {
         p3d_xformPoint(Tsupport, points[face_indices[0] - 1], triangle.p1);
         p3d_xformPoint(Tsupport, points[face_indices[1] - 1], triangle.p2);
         p3d_xformPoint(Tsupport, points[face_indices[2] - 1], triangle.p3);
         triangle.description= GP_DESCRIPTION_POINTS;
         htris.push_back(triangle);
      }
      else
      {
        printf("%s: %d: gpFind_placements_on_object(): the faces of \"%s\" should all be triangles.\n", __FILE__,__LINE__,support->name);
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
  double d, distToEnv, distToRobots, max;
  p3d_vector3 contact, zAxis={0.0,0.0,1.0};
  p3d_vector3 closest_points[2];
  p3d_matrix4 T1, T2;
  p3d_vector3 position;


  placementListIn.sort();
  placementListIn.reverse();
  while(placementListIn.size()>10)
  { placementListIn.pop_back();  }

  // for each horizontal triangle:
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
        p3d_mat4Rot(T1, zAxis, theta);

        position[0]= trisamples[i][0];
        position[1]= trisamples[i][1];
        position[2]= trisamples[i][2] + 0.005;

        T1[0][3]= position[0];
        T1[1][3]= position[1];
        T1[2][3]= position[2];

        //for each initial placement:
        for(iterP=placementListIn.begin(); iterP!=placementListIn.end(); iterP++)
        {
           //place at the current position and orientation:
           p3d_matMultXform(T1, (*iterP).T, T2);

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
           if(outside==false) 
           {  T2[2][3]+= 0.001;  }
           
           p3d_set_freeflyer_pose(object, T2);

           distToEnv= p3d_col_robot_environment_distance(object, closest_points[0], closest_points[1]);
           for(i=0; i<XYZ_ENV->nr; ++i)
           {
             if( XYZ_ENV->robot[i]==object || XYZ_ENV->robot[i]==support )
             {  continue;  }
             d=  p3d_col_robot_robot_distance(object, XYZ_ENV->robot[i], closest_points[0], closest_points[1]);
             if(d < distToRobots) 
             { distToRobots= d; }
           }
           d= MIN(distToEnv,distToRobots);
           
           if( p3d_col_test_robot(object, 0) )
           { continue; }
 
           placement= (*iterP);
           placement.theta= theta;
           p3d_vectCopy(position, placement.position);
           placement.clearance= d;
           placementListOut.push_back(placement);
        }
      
      }
    }

    free(trisamples);
    trisamples= NULL; 
  }

  placementListOut.sort(gpCompareClearance); //sort from the smallest to the biggest stability
  placementListOut.reverse(); //reverse the order of the elements in the list
  max= placementListOut.front().stability;

  // normalize the clearance score:
  i=1;
  for(iterP= placementListOut.begin(); iterP!=placementListOut.end(); iterP++)
  {  
    (*iterP).ID= i++;
    (*iterP).clearance/= max;
  }


  return GP_OK;
}



//! @ingroup stablePlacementComputation 
//! Finds, for a given mobile base configuration of the robot, a placement from the given placement list, that is
//! reachable by the arm and hand, and computes for the placement a configuration for the whole robot.
//! It also computes  an intermediate configuration (a configuration slightly before placing the object)
//! \param robot the robot
//! \param object the object to place
//! \param graspList a list of grasps
//! \param arm_type the robot arm type
//! \param qbase a configuration of the robot (only the part corresponding to the mobile base will be used)
//! \param grasp the grasp used to hold the object
//! \param hand parameters of the hand
//! \param distance distance between the grasp and pregrasp configurations (meters)
//! \param qpreplacement the preplacement configuration (must have been allocated before)
//! \param qplacement the placement configuration (must have been allocated before)
//! \param placement the found object placement
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpFind_placement_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpPlacement> &placementList, gpArm_type arm_type, configPt qbase, gpGrasp &grasp, gpHand_properties &hand, double distance, configPt qpreplacement, configPt qplacement, gpPlacement &placement)
{
  #ifdef GP_DEBUG
   if(robot==NULL)
   {
     printf("%s: %d: gpFind_placement_from_base_configuration(): robot is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
   if(object==NULL)
   {
     printf("%s: %d: gpFind_placement_from_base_configuration(): object is NULL.\n",__FILE__,__LINE__);
     return GP_ERROR;
   }
  #endif

  std::list<gpPlacement>::iterator iplacement;
  p3d_vector3 zAxis= {0,0,1};
  p3d_matrix4 base_frame, inv_base_frame;
  p3d_matrix4 T1, Tobject0, Tobject1, Tobject2;
  p3d_matrix4 gframe_world1, gframe_world2, gframe_robot1, gframe_robot2, tmp;
  configPt q0= NULL; //to store the current configuration of the robot
  configPt result1= NULL, result2= NULL;


  q0= p3d_get_robot_config(robot);
  p3d_get_freeflyer_pose(object, Tobject0);

  //On met à jour la configuration du robot pour que sa base soit dans la configuration
  //souhaitée:
  p3d_set_and_update_this_robot_conf(robot, qbase);
  result1= p3d_alloc_config(robot);
  result2= p3d_alloc_config(robot);


  gpGet_arm_base_frame(robot, base_frame); //on recupere le repere de la base du bras
  p3d_matInvertXform(base_frame, inv_base_frame);


  //for each placement of the list:
  for(iplacement=placementList.begin(); iplacement!=placementList.end(); iplacement++)
  {
    // place the object to its placement:
    p3d_mat4Rot(T1, zAxis, iplacement->theta);
    T1[0][3]= iplacement->position[0];
    T1[1][3]= iplacement->position[1];
    T1[2][3]= iplacement->position[2];


    p3d_mat4Mult(T1, iplacement->T, Tobject1);
    p3d_set_freeflyer_pose(object, Tobject1);

    T1[2][3]+= distance;

    p3d_mat4Mult(T1, iplacement->T, Tobject2);


    p3d_mat4Mult(Tobject1, grasp.frame, gframe_world1); //passage repere objet -> repere monde
    p3d_mat4Mult(Tobject2, grasp.frame, gframe_world2); //passage repere objet -> repere monde

    p3d_mat4Mult(inv_base_frame, gframe_world1, gframe_robot1); //passage repere monde -> repere robot
    p3d_mat4Mult(inv_base_frame, gframe_world2, gframe_robot2); //passage repere monde -> repere robot

    switch(arm_type)
    {
      case GP_PA10:
        p3d_mat4Mult(gframe_robot1, hand.Tgrasp_frame_hand, tmp);
        p3d_mat4Mult(tmp, hand.Thand_wrist, gframe_robot1);

        p3d_mat4Mult(gframe_robot2, hand.Tgrasp_frame_hand, tmp);
        p3d_mat4Mult(tmp, hand.Thand_wrist, gframe_robot2);

        p3d_copy_config_into(robot, qbase, &result1);
        p3d_copy_config_into(robot, qbase, &result2);

        if( gpInverse_geometric_model_PA10(robot, gframe_robot1, result1)==GP_OK && gpInverse_geometric_model_PA10(robot, gframe_robot2, result2)==GP_OK )
        {
           p3d_set_and_update_this_robot_conf(robot, result1);
           gpSet_grasp_configuration(robot, grasp, 0);
           p3d_set_freeflyer_pose(object, Tobject1);

           gpOpen_hand(robot, hand);
            if(p3d_col_test())
            {  continue;  }

           p3d_set_freeflyer_pose(object, Tobject2);
           p3d_set_and_update_this_robot_conf(robot, result2);
           gpOpen_hand(robot, hand);
            if(p3d_col_test())
            {  continue;  }


           p3d_set_and_update_this_robot_conf(robot, q0);
           p3d_destroy_config(robot, q0);
           p3d_set_freeflyer_pose(object, Tobject0);

           p3d_copy_config_into(robot, result1, &qplacement);
           p3d_copy_config_into(robot, result2, &qpreplacement);
           placement= *iplacement;
           return GP_OK;
        }
      break;
      default:
          printf("%s: %d: gpFind_placement_from_base_configuration(): undefined or unimplemented arm type.\n",__FILE__,__LINE__);
          p3d_set_and_update_this_robot_conf(robot, q0);
          p3d_destroy_config(robot, q0);
          return GP_ERROR;
      break;
    }

  }

  p3d_set_and_update_this_robot_conf(robot, q0);
  p3d_destroy_config(robot, q0);
  p3d_set_freeflyer_pose(object, Tobject0);

  return GP_ERROR;
}
