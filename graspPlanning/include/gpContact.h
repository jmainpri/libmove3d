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
#ifndef GP_CONTACT_H
#define GP_CONTACT_H


#include "P3d-pkg.h"

//! @ingroup graspPlanning 
//! This class is used to describe the characteristics of the contact points of a grasp.
//! It is also used to describe the contact points of an object pose (class gpPose).
class gpContact
{
 public:
  unsigned int ID; /*!< ID of the contact */
  p3d_polyhedre *surface; /*!<  surface (object) of the contact */
  unsigned int face;    /*!< index of the face (that must be a triangle), in the structure p3d_polyhedre, where the contact is located (starts from 0) */
  unsigned int fingerID;  /*!< ID (starting from 1) of the finger that realizes the contact (finger 1, finger 2, etc.)*/
  p3d_vector3 position; /*!<  contact position given in the object's frame */
  p3d_vector3 normal; /*!< surface normal at the contact point (directed outside the object) */
  p3d_vector3 forceDirection;  /*!< direction of the force exerted by the finger */
  double mu;         /*!<  friction coefficient of the contact */
  p3d_vector3 baryCoords; /*!< barycentric coordinates (defined by the vertices of the triangle) of the contact */
  double curvature; /*!<  curvature of the object surface at the contact point */
  double score; /*!< a score that can be used for any ordering need */
 
  gpContact();
  ~gpContact();
  gpContact(const gpContact &contact);
  gpContact & operator=(const gpContact &contact);
  int draw(double cone_length, int cone_nb_slices= 10);
  int computeBarycentricCoordinates();
  int computeCurvature();
  double distanceToSharpEdge(double angleThreshold);
};

#endif

