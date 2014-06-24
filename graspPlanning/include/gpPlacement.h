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
#ifndef GP_PLACEMENT_H
#define GP_PLACEMENT_H


#include "../graspPlanning/include/graspPlanning.h"
#include "../graspPlanning/include/gpContact.h"

//! @defgroup stablePlacementComputation
//! @ingroup graspPlanning 
//! This module implements some classes to compute a set of stable placements for a body.


//! @ingroup graspPlanning 
//! The following enum is used by the gpTriangle class to know
//! how it is described.
typedef enum gpTriangle_description
{
  GP_DESCRIPTION_INDICES, /*!< the triangle is described by the indices of its vertices in a vertex array */
  GP_DESCRIPTION_POINTS,/*!< the triangle is described by the coordinates of its vertices */
  GP_DESCRIPTION_BOTH /*!< the triangle is described by both data types */
} gpTriangle_description;


//! @ingroup stablePlacementComputation 
//! A basic class to store triangles in STL containers.
class gpTriangle
{
  public:
   //! a triangle can be described:
   //!  -by indices in a point array (the user must know which one it is):
   unsigned int i1, i2, i3;
   //!  -or directly by the coordinates of its vertices:
   p3d_vector3 p1, p2, p3;
   gpTriangle_description description;

   gpTriangle();
   unsigned int operator [] (unsigned int i) const;
   unsigned int & operator [] (unsigned int i);
   gpTriangle(const gpTriangle &triangle);
   gpTriangle& operator=(const gpTriangle &triangle);
};



//! @ingroup stablePlacementComputation 
//! Class containing information about a stable placement of an object (plane of the placement, stability criterion,
//! contact points on the plane).
//! A placement contains a "absolute" transform matrix that is completely object-centered. It is computed once for all.
//! The transform of a placement is the transformation that must be applied to the object (from its default configuration)
//! to place it on the horizontal plane with the face contacting the plane centered on (0,0,0).
//! A placement also contains information about where is placed the object i.e. a position and a angle around the vertical axis.
class gpPlacement
{
 public:
  int ID;  /*!< ID number */
  p3d_plane plane; /*!< plane of the contact points of the placement (its normal points toward the object interior)*/
  p3d_vector3 center; /*!< center of the orthogonal projection of the object's center of mass onto the placement plane */
  double stability; /*!< stability score of the placement (this value is supposed to have been normalized) */
  double clearance; /*!< clearance score of the placement (this value is supposed to have been normalized) */

   //! object-centered transformation of the placement: the transformation to apply to the object (wrt its default configuration) to place its suppport plane horizontally. It will be placed so that the placement center is at position (0,0,0) and the support plane normal is equal to -Z-axis 
  p3d_matrix4 T;

  //! position of the placement
  p3d_vector3 position;
  //! rotation angle around the support plane normal 
  double theta; 

  p3d_polyhedre *polyhedron;  /*!< surface of the grasped object (must be consistent with the field  "surface" of the contacts)*/
  p3d_rob *object;  /*!< the object (a freeflyer robot) */
  std::string object_name;  /*!< name of the object */
  std::vector<gpContact> contacts; 

  gpPlacement();
  gpPlacement(const gpPlacement &placement);
  ~gpPlacement();
  gpPlacement & operator=(const gpPlacement &placement);
//  bool operator < (const gpPlacement &placement);
  int computePoseMatrix(p3d_matrix4 pose); /*!< computes the pose to give to the object */
  int print();
  int draw(double length);
};

inline bool operator < (const gpPlacement &placement1, const gpPlacement &placement2)
{
    double dstab, dclear;
    
    dstab = fabs(placement1.stability - placement2.stability);
    dclear= fabs(placement1.clearance - placement2.clearance);
    
    if(dstab > dclear)
    {
        return (placement1.stability < placement2.stability) ? true : false;
    }
    else
    {
        return (placement1.clearance < placement2.clearance) ? true : false;
    }
}

bool gpCompareStability(const gpPlacement &place1, const gpPlacement &place2); 
bool gpCompareClearance(const gpPlacement &place1, const gpPlacement &place2); 

extern int gpCompute_stable_placements(p3d_rob *object, std::list<gpPlacement> &placementList);

extern int gpFind_placements_on_object(p3d_rob *object, p3d_rob *support, std::list<p3d_rob*> robotList, std::list<gpPlacement> placementListIn, double translationStep, unsigned int nbOrientations, double verticalOffset, std::list<gpPlacement> &placementListOut);

extern int gpFind_placement_from_base_configuration(p3d_rob *robot, p3d_rob *object, std::list<gpPlacement> &placementList, gpArm_type arm_type, configPt qbase, class gpGrasp &grasp, gpHand_properties &hand, double distance, configPt qpreplacement, configPt qplacement, gpPlacement &placement);

extern int gpCompute_placement_clearances(p3d_rob *object, std::list<p3d_rob*> robotList, std::list<gpPlacement> &placementList);

extern int gpPlacement_on_support_filter(p3d_rob *object, p3d_rob *support, std::list<gpPlacement> placementListIn, std::list<gpPlacement> &placementListOut);

#endif



