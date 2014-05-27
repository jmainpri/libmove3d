/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */

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

   gpSAHandInfo();
};

extern int gpSAHfinger_forward_kinematics(double length1, double length2, double length3, double q1, double q2, double q3, p3d_vector3 position, p3d_vector3 normal);

extern int gpSAHfinger_outer_workspace(double length1, double length2, double length3, double dq, std::vector<gpVector3D> &points, std::vector<gpVector3D> &normals);

extern int gpDraw_SAHfinger_outer_workspace(gpSAHandInfo data, double dq);

extern int gpSAHfinger_workspace(double length1, double length2, double length3, double dq, std::vector<gpVector3D> &points);

extern int gpSAHfinger_workspace_approximation(gpSAHandInfo data, double dq, double dr, unsigned int nb_spheres_max, std::vector<gpSphere> &spheres);

extern void svdcmp(float **a, int m, int n, float w[], float **v);

extern void p3d_mat3SVD(p3d_matrix3 M, p3d_matrix3 U, p3d_vector3 S, p3d_matrix3 V);

extern void p3d_mat4SVD(p3d_matrix4 M, p3d_matrix4 U, p3d_vector4 S, p3d_matrix4 V);

extern int gpSAHfinger_jacobian(double length1, double length2, double length3, double q1, double q2, double q3, p3d_matrix3 J);

extern int gpSAHfinger_main_force_direction(p3d_matrix4 Twrist, gpHand_properties &handProp, double q[4], int finger_index, p3d_vector3 direction);

// extern int gpSAHfinger_manipulability_ellipsoid(double length1, double length2, double length3, double q1, double q2, double q3);

extern int gpDraw_SAHfinger_manipulability_ellipsoid(p3d_rob *robot, gpHand_properties &hand_properties, int finger_index, int handID= 0);

extern int gpDraw_reachable_points(p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp);

#endif
