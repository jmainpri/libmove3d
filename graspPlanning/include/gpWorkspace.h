
#ifndef GP_WORKSPACE_H
#define GP_WORKSPACE_H

//! @defgroup workspace
//! @ingroup graspPlanning 
//! This module implements some structures and functions to compute
//! the approximation of the workspace of a finger of the Schunk Anthropomorphic Hand.
//! The workspace is approximated as a set of spheres striclty included inside it.

//! @ingroup workspace 
//! A class to store geometrical information concerning the SAHand finger.
class gpSAHandInfo
{
  public:
   double q1min, q1max;
   double q2min, q2max;
   double q3min, q3max;
   //! lengths of the phalanges:
   double length1, length2, length3;

   gpSAHandInfo()
   {
//      q1min= -20*DEGTORAD;
//      q1max=  20*DEGTORAD;
//      q2min= -19*DEGTORAD;
//      q2max=  90*DEGTORAD;
//      q3min=   0*DEGTORAD;
//      q3max=  90*DEGTORAD;
     // these bounds are reduced because a finger has no chance to ensure a good contact
     // close to the initial bounds:
     q1min= -20*DEGTORAD;
     q1max=  20*DEGTORAD;
     q2min= 10*DEGTORAD;
     q2max=  70*DEGTORAD;
     q3min=   20*DEGTORAD;
     q3max=  60*DEGTORAD;

     length1= 0.067816;
     length2= 0.029980;
     length3= 0.029;
   }
};


extern int gpSAHfinger_forward_kinematics(float q1, float q2, float q3, float length1, float length2, float length3, p3d_vector3 position, p3d_vector3 normal);

extern int gpSAHfinger_outer_workspace(double length1, double length2, double length3, double dq, std::vector<gpVector3D> &points, std::vector<gpVector3D> &normals);

extern int gpDraw_SAHfinger_outer_workspace(gpSAHandInfo data, double dq);

extern int gpSAHfinger_workspace(double length1, double length2, double length3, double dq, std::vector<gpVector3D> &points);

extern int gpSAHfinger_workspace_approximation(gpSAHandInfo data, double dq, double dr, unsigned int nb_spheres_max, std::vector<gpSphere> &spheres);

extern void svdcmp(float **a, int m, int n, float w[], float **v);

extern void p3d_mat3SVD(p3d_matrix3 M, p3d_matrix3 U, p3d_vector3 S, p3d_matrix3 V);

extern void p3d_mat4SVD(p3d_matrix4 M, p3d_matrix4 U, p3d_vector4 S, p3d_matrix4 V);

#endif
