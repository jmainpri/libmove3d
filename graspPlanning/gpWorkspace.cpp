
#include "GraspPlanning-pkg.h"
#include <vector>

//! @ingroup workspace 
//! Computes the position of the fingertip center point of the Schunk Anthropomorphic Hand finger.
//! \param length1 length of the first phalanx
//! \param length2 length of the second phalanx
//! \param length3 length of the third phalanx
//! \param q1 first joint parameter (controls the (palm)-(proximal phalanx) joint first DOF (abduction))
//! \param q2 second joint (controls the (palm)-(proximal phalanx) joint second DOF (subduction))
//! \param q3 third joint parameter (controls the DOFs of the last two joints that are coupled)
//! \param position the computed position of the fingertip center
//! \param normal a vector giving the direction of the fingertip contact surface (orthogonal to the medial axis of the distal phalanx and directed towards the inside of the hand)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_forward_kinematics(double length1, double length2, double length3, double q1, double q2, double q3, p3d_vector3 position, p3d_vector3 normal)
{
  position[0]= -sin(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  ); 
  position[1]=  cos(q1)*(  length1*cos(q2) + length2*cos(q2+q3) + length3*cos(q2+2*q3)  );
  position[2]= -length1*sin(q2) - length2*sin(q2+q3) - length3*sin(q2+2*q3);

  normal[0]=  sin(q1)*(sin(q2 + 2*q3));
  normal[1]= -cos(q1)*(sin(q2 + 2*q3));
  normal[2]= -cos(q2 + 2*q3);

  return GP_OK;
}


//! @ingroup workspace 
//! Computes the outer envelope of the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \param points computed point cloud
//! \param normal fingertip surface normal at each point of the workspace 
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_outer_workspace(gpSAHandInfo data, double dq, std::vector<gpVector3D> &points, std::vector<gpVector3D> &normals)
{
  unsigned int i, j, n1, n2, n3;
  double q1, q2, q3;
  double q1min, q1max, q2min, q2max, q3min, q3max;
  p3d_vector3 p, n;
  gpVector3D point, normal;
  std::list<gpVector3D> pointList, normalList;
  std::list<gpVector3D>::iterator iter;

  q1min= data.q1min;
  q1max= data.q1max;
  q2min= data.q2min;
  q2max= data.q2max;
  q3min= data.q3min;
  q3max= data.q3max;

  n1= (unsigned int) ( (q1max-q1min)/dq );
  n2= (unsigned int) ( (q2max-q2min)/dq );
  n3= (unsigned int) ( (q3max-q3min)/dq );

  for(i=0; i<=n1; ++i)
  {
    q1= q1min + i*dq;

    for(j=0; j<=n3; ++j)
    {
      q3= q3min + j*dq;

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2min, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2max, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }  

    for(j=0; j<=n2; ++j)
    {
      q2= q2min + j*dq;
      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3min, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3max, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }
  }

  for(i=0; i<=n2; ++i)
  {
    for(j=0; j<=n3; ++j)
    {
      q2= q2min + i*dq;
      q3= q3min + j*dq;
      q1= q1min;
      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1min, q2, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);

      gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1max, q2, q3, p, n);
      point.x= p[0];
      point.y= p[1];
      point.z= p[2];
      pointList.push_back(point);
      normal.x= n[0];
      normal.y= n[1];
      normal.z= n[2];
      normalList.push_back(normal);
    }
  }

  points.resize(pointList.size());
  i= 0;
  for(iter=pointList.begin(); iter!=pointList.end(); iter++)
  {
    points.at(i)= *iter;
    i++;
  }
  normals.resize(normalList.size());
  i= 0;
  for(iter=normalList.begin(); iter!=normalList.end(); iter++)
  {
    normals.at(i)= *iter;
    i++;
  }

  return GP_OK;
}


//! @ingroup workspace 
//! Draws the outer envelope of the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpDraw_SAHfinger_outer_workspace(gpSAHandInfo data, double dq)
{
  std::vector<gpVector3D> points, normals;

  gpSAHfinger_outer_workspace(data, dq, points, normals);

  unsigned int i;

  glPushAttrib(GL_POINT_BIT);
  glPointSize(4);
glEnable(GL_NORMALIZE);
  glBegin(GL_POINTS);
   for(i=0; i<points.size(); ++i)
   {
//      glNormal3d(normals[i].x, normals[i].y, normals[i].z);
     glNormal3d(points[i].x, points[i].y, points[i].z);
     glVertex3d(points[i].x, points[i].y, points[i].z);
   }
  glEnd();

//   glColor3f(1,0,0);
//   glBegin(GL_LINES);
//    for(i=0; i<points.size(); ++i)
//    {
//      glVertex3d(points[i].x, points[i].y, points[i].z);
//      glVertex3d(points[i].x+0.01*normals[i].x, points[i].y+0.01*normals[i].y, points[i].z+0.01*normals[i].z);
//    }
//   glEnd();

  glPopAttrib();

  return GP_OK;
}


//! @ingroup workspace 
//! Computes the workspace of a SAHand finger as a point cloud.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \param points computed point cloud
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_workspace(gpSAHandInfo data, double dq, std::vector<gpVector3D> &points)
{
  unsigned int i, j, k, n1, n2, n3;
  double q1, q2, q3;
  double q1min, q1max, q2min, q2max, q3min, q3max;
  p3d_vector3 p, n;
  gpVector3D point;
  std::list<gpVector3D> pointList;
  std::list<gpVector3D>::iterator iter;

  q1min= data.q1min;
  q1max= data.q1max;
  q2min= data.q2min;
  q2max= data.q2max;
  q3min= data.q3min;
  q3max= data.q3max;

  n1= (unsigned int) ( (q1max-q1min)/dq );
  n2= (unsigned int) ( (q2max-q2min)/dq );
  n3= (unsigned int) ( (q3max-q3min)/dq );

  for(i=0; i<=n1; ++i)
  {
    q1= q1min + i*dq;
    for(j=0; j<=n2; ++j)
    {
      q2= q2min + j*dq;
      for(k=0; k<=n3; ++k)
      {
        q3= q3min + k*dq;
        gpSAHfinger_forward_kinematics(data.length1, data.length2, data.length3, q1, q2, q3, p, n);
        point.x= p[0];
        point.y= p[1];
        point.z= p[2];
        pointList.push_back(point);
      }
    }  
  }

  points.resize(pointList.size());
  i= 0;
  for(iter=pointList.begin(); iter!=pointList.end(); iter++)
  {
    points.at(i)= *iter;
    i++;
  }

  return GP_OK;
}


//! @ingroup workspace 
//! Computes an approximation of the SAHand finger workspace as a set of spheres.
//! All the spheres are completely included in the real workspace.
//! The function builds a set of points inside the workspace (inner points) and a set of points
//! on the workspace boundary (boundary points).
//! The minimal distance from each inner point to the boundary points is computed.
//! The first sphere center is the inner point with the maximal distance (that will be the sphere radius).
//! All the inner points that are inside the sphere are removed. 
//! The new distance of each remaining points is the minimum of the old distance and the distance
//! from the point to the sphere.
//! The new sphere center is the inner point with the maximal distance and the algorithm continues
//! with the same principle until the maximal number is reached ro the last computed sphere radius
//! is smaller than the given threshold.
//! \param data geometrical info about the finger
//! \param dq discretization step of the joint parameters (in radians)
//! \param dr the algorithm will end when it has found a sphere with radius < dr
//! \param nb_spheres_max maximal number of spheres that will be computed
//! \param spheres computed sphere set
//! \return GP_OK in case of success, GP_ERROR otherwise
int gpSAHfinger_workspace_approximation(gpSAHandInfo data, double dq, double dr, unsigned int nb_spheres_max, std::vector<gpSphere> &spheres)
{
  unsigned int i, j;
  double distance, dmin;
  gpVector3D best;
  gpSphere sphere;
  std::vector<gpVector3D> inner, boundary, normals;
  std::list<gpVector3D> innerList;
  std::list<gpVector3D>::iterator iterPoint;
  std::list<gpSphere> sphereList;
  std::list<gpSphere>::iterator iterSphere;

  gpSAHfinger_workspace(data, dq, inner);
  gpSAHfinger_outer_workspace(data, dq, boundary, normals);

  for(i=0; i<inner.size(); ++i)
  {
    for(j=0; j<boundary.size(); ++j)
    {
      distance= sqrt( pow(inner[i].x-boundary[j].x, 2) + pow(inner[i].y-boundary[j].y, 2) + pow(inner[i].z-boundary[j].z, 2) );
      if(distance < dmin || j==0)
      {
        dmin= distance;
        inner[i].cost= distance;
      }
    }
    innerList.push_back(inner[i]);
  }

  innerList.sort();

  while(true)
  {
    best= innerList.back();
    sphere.center[0]= best.x;
    sphere.center[1]= best.y;
    sphere.center[2]= best.z;
    sphere.radius   = best.cost;
    
    sphereList.push_back(sphere);
    if(sphere.radius < dr || sphereList.size() > nb_spheres_max)
    {  break;  }

    innerList.pop_back();

    //removes points that are inside the sphere and computes the noew distance:
    iterPoint= innerList.begin();
    while(iterPoint!=innerList.end())
    {
      distance= sqrt( pow(iterPoint->x-sphere.center[0], 2) + pow(iterPoint->y-sphere.center[1], 2) + pow(iterPoint->z-sphere.center[2], 2) );
      if( distance <= sphere.radius)
      {
        iterPoint= innerList.erase(iterPoint);
        continue;
      }
      if( (distance-sphere.radius) < iterPoint->cost )
      {
        iterPoint->cost= (distance-sphere.radius);
      }
      iterPoint++;
    }
    innerList.sort();
  }

  spheres.resize(sphereList.size());
  i= 0;
  for(iterSphere=sphereList.begin(); iterSphere!=sphereList.end(); iterSphere++)
  {
    spheres.at(i)= *iterSphere;
    printf("       workspace.at(%d).setCenter(%f, %f, %f); \n", i, spheres.at(i).center[0], spheres.at(i).center[1], spheres.at(i).center[2]);
    printf("       workspace.at(%d).radius= %f; \n", i, spheres.at(i).radius);

    i++;
  }


  return GP_OK;
}


